import math
import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

KP = 0.8
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1

# Corner-cutting fix (Corolla Cross rlog 2026-09-21: the e2e path sits ~0.5 m inside on curves). Relax the
# commanded curvature by a fraction of the part above a deadzone, so the car runs a little wider (outward) in
# proportion to how tight the curve is. Straights and small lane corrections below the deadzone are untouched.
CURVE_OUTWARD_DEADZONE = 0.0005  # 1/m (~radius 2000 m): below this is a straight / small correction, untouched.
                                 # Lowered from 0.0012 (radius 830 m) on 2026-09-25: a gentle right curve at a set
                                 # 100 km/h still ran inside because its curvature sat under the old deadzone, so the
                                 # bias never applied there no matter the fraction.
# The outward bias only helps at higher speed, where the path cuts the inside. On tight LOW-speed curves the car
# tends to run WIDE instead, so relaxing the curvature there makes it worse. Speed schedule (m/s -> fraction):
# 0 up to 40 km/h, 0.09 at 70, 0.155 at 90 km/h, 0.19 at 120 km/h (road test 2026-09-25: 90 km/h centred OK, 107 km/h still
# cut the inside).
CURVE_OUTWARD_V_BP = [11.1, 19.4, 25.0, 33.3]   # m/s (40, 70, 90, 120 km/h)
CURVE_OUTWARD_FRAC_V = [0.0, 0.09, 0.155, 0.19]


def apply_curve_outward_bias(desired_curvature: float, v_ego: float) -> float:
  # In a fast curve, command a slightly smaller curvature so the car runs wider and stops cutting the inside. Faded
  # out at low speed (tight curves there need the full turn-in or the car runs wide). Straights and small lane
  # corrections below the deadzone are untouched.
  frac = float(np.interp(v_ego, CURVE_OUTWARD_V_BP, CURVE_OUTWARD_FRAC_V))
  if frac <= 0.0 or abs(desired_curvature) <= CURVE_OUTWARD_DEADZONE:
    return desired_curvature
  return desired_curvature * (1.0 - frac)

# Lane centering feedback (Corolla Cross rlog 2026-09-25, route 4d seg 9): in a 70 m-radius curve at 47 km/h the car sat
# ~1 m inside the lane (riding the inner line) although the torque loop tracked the model's desired curvature almost
# exactly (14.2 vs 14.4 e-3). The e2e action itself holds the car inside, and the outward bias above is a blind
# percentage. Close the loop on the lane lines instead: measure the car's offset from the lane centre and add a small
# curvature correction that walks it back to the middle. Conventions (verified on seg 1): model y is positive to the
# RIGHT and curvature is positive to the RIGHT, so a car left of centre ((yl + yr) / 2 > 0) needs a positive
# correction. The correction is a lateral-acceleration request, so the same gain gives the same feel at any speed.
# PD on offset + lane heading (damping), plus a slow capped integral for what the model keeps pulling. Simulated
# (bicycle model, 0.3 s steering lag, 2 cm line noise): 1 m inside at 47 km/h in a 70 m curve is back within
# 10 cm in ~8 s with no overshoot, peak correction 1.9e-3 (0.32 m/s^2); 20 cm at 90 km/h in ~10 s.
LANE_CENTER_MIN_SPEED = 8.0        # m/s (~30 km/h): below this the lines are too close / too curved to trust
# Only at highway speed. Route 55 seg 8 (2026-09-26, 33-47 km/h right after the driver released the wheel out of a
# turn): the correction sat on its cap (0.6 m/s^2 = 3.5e-3 1/m at 12 m/s, ~30% of the model's own swing) in phase
# with the model's correction, and the car swung left/right with a ~3 s period for 10 s. The driver wants low speed
# left exactly as stock; the feedback fades in between 70 and 80 km/h.
LANE_CENTER_FADE_BP = [19.4, 22.2]  # m/s (70, 80 km/h)
LANE_CENTER_FADE_V = [0.0, 1.0]
LANE_CENTER_MIN_LINE_PROB = 0.5    # the better of the two near lane lines must be at least this confident
LANE_CENTER_MIN_OTHER_PROB = 0.4   # ...and the weaker one at least this. 0.3 let a lost outer line (prob 0.31-0.41, route 58
                                   # seg 6) put the lane centre 1 m off while the car was actually centred; in the
                                   # inside-cutting curves the outer line sits at 0.41-0.61
LANE_CENTER_WIDTH_CONF_PROB = 0.6  # both lines at least this to teach the width filter
LANE_CENTER_WIDTH_TAU = 3.0        # s; filter of the lane width, fed only by confident frames
LANE_CENTER_WIDTH_TOL = 0.7        # m; a width this far from the filtered one means one line is not the ego lane's
LANE_CENTER_WIDTH_RANGE = (2.6, 4.6)  # m; outside this the pair is not the ego lane
LANE_CENTER_FIT_RANGE = 12.0       # m ahead used to fit the lane centre (offset + heading + curvature)
LANE_CENTER_DEADBAND = 0.03        # m; no position correction inside this
LANE_CENTER_KP = 0.35              # m/s^2 of lateral accel per metre of offset
LANE_CENTER_KD = 1.20              # m/s^2 per m/s of lateral speed toward/away from the centre (damping, ~critical)
LANE_CENTER_KI = 0.05              # m/s^2 per metre per second: slowly takes over what the model keeps pulling
LANE_CENTER_I_LIMIT = 0.25         # m/s^2; cap on the integrated part
LANE_CENTER_MAX_LAT_ACCEL = 0.6    # m/s^2; cap on the total correction as felt by the driver
LANE_CENTER_MAX_CURV = 3.5e-3      # 1/m; cap on the total correction (radius ~290 m)
LANE_CENTER_JERK = 0.5             # m/s^3; how fast the correction may change, as lateral jerk so it feels the same at
                                   # every speed (driver 2026-09-26: corrections at ~90 km/h felt stiff; was 3e-3 1/m/s = 1.9 m/s^3 there)
LANE_CENTER_FILTER_TAU = 0.5       # s; low-pass on the measured offset and heading


class LaneCentering:
  def __init__(self, dt: float):
    self.dt = dt
    self.offset_filter = FirstOrderFilter(0.0, LANE_CENTER_FILTER_TAU, dt)
    self.heading_filter = FirstOrderFilter(0.0, LANE_CENTER_FILTER_TAU, dt)
    self.width_filter = FirstOrderFilter(0.0, LANE_CENTER_WIDTH_TAU, dt)  # x == 0 until the first confident width
    self.integral = 0.0
    self.correction = 0.0
    self.offset = 0.0
    self.heading = 0.0
    self.valid = False
    self.last_meas: tuple[float, float] | None = None  # this frame's (offset, heading), for the low-speed trim

  def reset(self):
    self.offset_filter.x = 0.0
    self.heading_filter.x = 0.0
    self.integral = 0.0
    self.correction = 0.0
    self.valid = False

  def measure(self, model_v2) -> tuple[float, float] | None:
    """(offset, heading) of the car relative to the lane centre: offset positive = car LEFT of centre, heading
    positive = car pointing LEFT of the lane direction. None when the lines are unreliable."""
    if model_v2 is None:
      return None
    lines = model_v2.laneLines
    probs = model_v2.laneLineProbs
    if len(lines) < 4 or len(probs) < 4 or len(lines[1].y) < 4 or len(lines[2].y) < 4:
      return None
    if max(probs[1], probs[2]) < LANE_CENTER_MIN_LINE_PROB or min(probs[1], probs[2]) < LANE_CENTER_MIN_OTHER_PROB:
      return None
    yl, yr = lines[1].y[0], lines[2].y[0]
    width = yr - yl
    if not (LANE_CENTER_WIDTH_RANGE[0] <= width <= LANE_CENTER_WIDTH_RANGE[1]):
      return None
    if self.width_filter.x > 0.0 and abs(width - self.width_filter.x) > LANE_CENTER_WIDTH_TOL:
      return None  # one of the two lines is not this lane's (a lost outer line placed somewhere else)
    if min(probs[1], probs[2]) >= LANE_CENTER_WIDTH_CONF_PROB:
      if self.width_filter.x > 0.0:
        self.width_filter.update(width)
      else:
        self.width_filter.x = width
    x = np.asarray(lines[1].x)
    n = int(np.sum(x <= LANE_CENTER_FIT_RANGE))
    if n < 4:
      return None
    centre = (np.asarray(lines[1].y)[:n] + np.asarray(lines[2].y)[:n]) / 2.0  # model y: positive = right (capnp lists do not slice)
    # centre(x) ~ c0 + c1 x + c2 x^2: c0 is the centre's lateral position (right of the car), c1 its slope. A lane
    # that runs to the right ahead (c1 > 0) means the car is pointing LEFT of it.
    c2, c1, c0 = np.polyfit(x[:n], centre, 2)
    return float(c0), float(math.atan(c1))

  def update(self, model_v2, v_ego: float, active: bool, steering_pressed: bool) -> float:
    """Curvature to ADD to the desired curvature (positive = right)."""
    meas = self.measure(model_v2) if active and not steering_pressed and v_ego > LANE_CENTER_MIN_SPEED else None
    self.last_meas = meas
    step = LANE_CENTER_JERK / max(v_ego, LANE_CENTER_MIN_SPEED) ** 2 * self.dt
    if meas is None:
      # lines lost or driver steering: wind the correction back to zero at the same jerk instead of snapping (the
      # snap showed up as a 3.9 m/s^3 spike when replaying route 53 seg 15)
      self.offset_filter.x = 0.0
      self.heading_filter.x = 0.0
      self.integral = 0.0
      self.valid = False
      self.correction = float(np.clip(0.0, self.correction - step, self.correction + step))
      return self.correction

    self.valid = True
    self.offset = self.offset_filter.update(meas[0])
    self.heading = self.heading_filter.update(meas[1])
    error = self.offset - float(np.clip(self.offset, -LANE_CENTER_DEADBAND, LANE_CENTER_DEADBAND))  # deadband
    lateral_speed = v_ego * math.sin(self.heading)  # positive = drifting LEFT

    self.integral = float(np.clip(self.integral + LANE_CENTER_KI * error * self.dt, -LANE_CENTER_I_LIMIT, LANE_CENTER_I_LIMIT))
    lat_accel = LANE_CENTER_KP * error + LANE_CENTER_KD * lateral_speed + self.integral
    lat_accel = float(np.clip(lat_accel, -LANE_CENTER_MAX_LAT_ACCEL, LANE_CENTER_MAX_LAT_ACCEL))
    target = float(np.clip(lat_accel / max(v_ego, LANE_CENTER_MIN_SPEED) ** 2, -LANE_CENTER_MAX_CURV, LANE_CENTER_MAX_CURV))
    target *= float(np.interp(v_ego, LANE_CENTER_FADE_BP, LANE_CENTER_FADE_V))
    self.correction = float(np.clip(target, self.correction - step, self.correction + step))
    return self.correction


# Low-speed lane trim (below the highway-speed centering). Route 58 seg 4 and route 4d segs 9/11 (Corolla Cross,
# 2026-09-25/27, 45-55 km/h, 70-100 m curves): when the OUTER (dashed) line is seen weakly the model hugs the inner
# solid line at ~1.0 m from the car's centre line whatever the lane width (3.7-4.1 m), i.e. the car rides the inner
# line; when both lines are seen clearly (route 58 seg 6) it centres. The fast PD centering oscillated at these
# speeds (route 55 seg 8), so this is an INTEGRAL-ONLY trim: a sustained offset slowly builds a small lateral-accel
# bias, capped low, that the model's own fast loop rides on top of. It fades out where the highway centering fades in.
LANE_TRIM_MIN_SPEED = 8.0        # m/s (~30 km/h)
LANE_TRIM_FADE_BP = [19.4, 22.2]  # m/s: full below 70 km/h, gone at 80 (the highway centering takes over)
LANE_TRIM_FADE_V = [1.0, 0.0]
LANE_TRIM_ENGAGE_OFFSET = 0.30   # m; the offset must exceed this ...
LANE_TRIM_ENGAGE_TIME = 1.0      # s; ... for this long before the trim starts building
LANE_TRIM_KI = 0.08              # m/s^2 per metre per second while the offset keeps the trim's direction
LANE_TRIM_UNWIND_KI = 0.24       # m/s^2 per metre per second once the car is past the centre (3x, so it does not overshoot)
LANE_TRIM_RELEASE_OFFSET = 0.15  # m; inside this band the trim is released ...
LANE_TRIM_RELEASE_DECAY = 0.15   # m/s^2 per s; ... at this rate
LANE_TRIM_FLIP_CURV = 1.0e-3     # 1/m; the curve direction has flipped when the desired curvature is this far the other way
LANE_TRIM_FLIP_DECAY = 0.30      # m/s^2 per s; a trim built in a left curve is bled off quickly in the following right curve
LANE_TRIM_MAX_LAT_ACCEL = 0.25   # m/s^2; ~12% of the curve's own lateral accel at 50 km/h in a 70 m curve
LANE_TRIM_HOLD_TIME = 3.0        # s; lines lost: hold the trim this long (the outer line flickers in these curves) ...
LANE_TRIM_DECAY = 0.10           # m/s^2 per s; ... then bleed it off
LANE_TRIM_OVERRIDE_DECAY = 0.5   # m/s^2 per s; driver steering / inactive: bleed it off quickly
LANE_TRIM_JERK = 0.3             # m/s^3; rate limit on the resulting curvature, as lateral jerk


class LaneTrimLowSpeed:
  def __init__(self, dt: float, centering: LaneCentering):
    self.dt = dt
    self.centering = centering  # shares its lane-geometry measurement (made once per frame in its update)
    self.offset_filter = FirstOrderFilter(0.0, LANE_CENTER_FILTER_TAU, dt)
    self.lat_accel = 0.0
    self.correction = 0.0
    self.engage_t = 0.0
    self.engage_curv = 0.0  # desired curvature when the trim started building (to notice a curve-direction flip)
    self.lost_t = 0.0
    self.valid = False

  def update(self, model_v2, v_ego: float, active: bool, steering_pressed: bool, desired_curvature: float = 0.0) -> float:
    """Curvature to ADD to the desired curvature (positive = right)."""
    usable = active and not steering_pressed and v_ego > LANE_TRIM_MIN_SPEED
    meas = self.centering.last_meas if usable else None
    self.valid = meas is not None

    if self.lat_accel != 0.0 and self.engage_curv * desired_curvature < 0.0 and abs(desired_curvature) > LANE_TRIM_FLIP_CURV:
      # the curve has changed direction since the trim was built: it is stale, bleed it off fast
      self.lat_accel = float(np.clip(0.0, self.lat_accel - LANE_TRIM_FLIP_DECAY * self.dt, self.lat_accel + LANE_TRIM_FLIP_DECAY * self.dt))
      self.engage_t = 0.0
      if self.lat_accel == 0.0:
        self.engage_curv = 0.0

    if meas is not None:
      self.lost_t = 0.0
      offset = self.offset_filter.update(meas[0])  # + = car LEFT of centre -> needs a positive (right) lat accel
      if abs(offset) > LANE_TRIM_ENGAGE_OFFSET:
        self.engage_t = min(self.engage_t + self.dt, LANE_TRIM_ENGAGE_TIME)
      else:
        self.engage_t = max(self.engage_t - self.dt, 0.0)
      engaged = self.engage_t >= LANE_TRIM_ENGAGE_TIME
      if engaged and self.lat_accel == 0.0:
        self.engage_curv = desired_curvature
      if abs(offset) < LANE_TRIM_RELEASE_OFFSET:
        self.lat_accel = float(np.clip(0.0, self.lat_accel - LANE_TRIM_RELEASE_DECAY * self.dt, self.lat_accel + LANE_TRIM_RELEASE_DECAY * self.dt))
      elif engaged or self.lat_accel != 0.0:
        ki = LANE_TRIM_UNWIND_KI if offset * self.lat_accel < 0.0 else LANE_TRIM_KI
        self.lat_accel = float(np.clip(self.lat_accel + ki * offset * self.dt, -LANE_TRIM_MAX_LAT_ACCEL, LANE_TRIM_MAX_LAT_ACCEL))
    else:
      self.offset_filter.x = 0.0
      self.engage_t = 0.0
      decay = LANE_TRIM_OVERRIDE_DECAY if not usable else 0.0
      if usable:
        self.lost_t += self.dt
        if self.lost_t > LANE_TRIM_HOLD_TIME:
          decay = LANE_TRIM_DECAY
      self.lat_accel = float(np.clip(0.0, self.lat_accel - decay * self.dt, self.lat_accel + decay * self.dt))
    if self.lat_accel == 0.0:
      self.engage_curv = 0.0

    v2 = max(v_ego, LANE_TRIM_MIN_SPEED) ** 2
    target = self.lat_accel / v2 * float(np.interp(v_ego, LANE_TRIM_FADE_BP, LANE_TRIM_FADE_V))
    step = LANE_TRIM_JERK / v2 * self.dt
    self.correction = float(np.clip(target, self.correction - step, self.correction + step))
    return self.correction


# Lateral jerk cap on the model's desired curvature. The MACROSTIFF model's path corrections felt a bit stiff to the
# driver (route 53, 2026-09-26). Measured above 60 km/h outside lane changes on 2026-09-25/26: |d(desired curvature)/dt|
# * v^2 has p90 0.70, p95 1.22, p99 2.19 m/s^3. Capping at 1.0 m/s^3 touches ~7% of frames - only the sharpest
# corrections, which then reach the same curvature about 0.1-0.2 s later - and leaves steady centering untouched.
# Bypassed during lane changes (controlsd has its own start-rate cap there) and when lateral control is inactive.
DESIRED_CURVATURE_MAX_LAT_JERK = 1.0  # m/s^3
DESIRED_CURVATURE_JERK_MIN_SPEED = 8.0  # m/s; below this the cap in curvature terms is so loose it barely acts
# Highway speed only, like the lane centering: below 70 km/h the cap is effectively off (8 m/s^3), fading to 1.0 at 80.
DESIRED_CURVATURE_JERK_FADE_BP = [19.4, 22.2]  # m/s
DESIRED_CURVATURE_JERK_FADE_V = [8.0, DESIRED_CURVATURE_MAX_LAT_JERK]


class DesiredCurvatureJerkLimiter:
  def __init__(self, dt: float):
    self.dt = dt
    self.prev = 0.0

  def update(self, desired_curvature: float, v_ego: float, active: bool, lane_changing: bool) -> float:
    if not active or lane_changing:
      self.prev = desired_curvature
      return desired_curvature
    max_jerk = float(np.interp(v_ego, DESIRED_CURVATURE_JERK_FADE_BP, DESIRED_CURVATURE_JERK_FADE_V))
    step = max_jerk / max(v_ego, DESIRED_CURVATURE_JERK_MIN_SPEED) ** 2 * self.dt
    self.prev = float(np.clip(desired_curvature, self.prev - step, self.prev + step))
    return self.prev


class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)
    self.lane_centering = LaneCentering(dt)
    self.lane_trim = LaneTrimLowSpeed(dt, self.lane_centering)
    self.curvature_jerk_limiter = DesiredCurvatureJerkLimiter(dt)

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # Override torque params from extension
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    model_curvature = desired_curvature
    desired_curvature = apply_curve_outward_bias(desired_curvature, CS.vEgo)
    desired_curvature += self.lane_centering.update(self.extension.model_v2, CS.vEgo, active, CS.steeringPressed)
    desired_curvature += self.lane_trim.update(self.extension.model_v2, CS.vEgo, active, CS.steeringPressed, model_curvature)
    lane_changing = self.extension.model_v2 is not None and self.extension.model_v2.meta.laneChangeState != 0
    desired_curvature = self.curvature_jerk_limiter.update(desired_curvature, CS.vEgo, active, lane_changing)
    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    error = setpoint - measurement

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len+1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx+1] - self.lat_accel_request_buffer[lookahead_idx-1]) / (2 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    ff = gravity_adjusted_future_lateral_accel
    # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
    ff -= self.torque_params.latAccelOffset
    ff += get_friction(error + JERK_GAIN * desired_lateral_jerk, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      # Lateral acceleration torque controller extension updates
      # Overrides pid_log.error and output_torque
      pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
                                                     future_desired_lateral_accel, measurement, lateral_accel_deadzone, gravity_adjusted_future_lateral_accel,
                                                     desired_curvature, measured_curvature, steer_limited_by_safety, output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
