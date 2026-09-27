import math
import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY, CV
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
# 15-30 km/h gains halved for the Corolla Altis Hybrid (2026-09-27, route b7 seg 8): with the stock schedule
# (11.5 / 5.5 / 3.5 at 5 / 7.5 / 10 m/s) a wheel release mid-turn at 16 km/h turned into a full-authority limit cycle -
# torque saturated 71% of the time, P term +-4 on a 0.3 m/s^2 error, ~1.2 s period, steering +-40-56 deg.
KP_INTERP = [250, 120, 65, 30, 6.0, 3.2, 2.0, 1.2, KP]  # 10/15 m/s 2.4/1.6 -> 2.0/1.2 (2026-09-27 21:13: 40-50 km/h curve-exit sway)

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1

# Corner-cutting fix (Altis rlog 2026-09-20: the e2e path sits ~0.6 m inside on curves, both directions). Relax the
# commanded curvature by a fraction so the car runs a little wider (outward) in proportion to how tight the curve is.
# Straights and small lane corrections below the deadzone are untouched. Schedule ported from tncr18 (Corolla Cross
# road tests 2026-09-23..25): the outward bias only helps at higher speed, where the path cuts the inside. On tight
# LOW-speed curves the car tends to run WIDE instead, so relaxing the curvature there makes it worse.
CURVE_OUTWARD_DEADZONE = 0.0005  # 1/m (~radius 2000 m): below this is a straight / small correction, untouched.
# Speed schedule (m/s -> fraction): 0 up to 40 km/h, 0.09 at 70, 0.155 at 90 km/h, 0.19 at 120 km/h.
CURVE_OUTWARD_V_BP = [v * CV.KPH_TO_MS for v in (30., 40., 50., 70., 90., 100., 120.)]   # m/s
# Negative = INWARD bias (command more curvature than the model): Altis driver 2026-09-27, 30-40 km/h curves want more
# steering; -0.05 at 30 km/h, back to 0 at 40, then the outward schedule. Stays 0 exactly at 40 so the two never fight.
CURVE_OUTWARD_FRAC_V = [-0.05, 0.0, 0.05, 0.09, 0.155, 0.167, 0.167]  # flat from 100 km/h (was 0.19 at 120), 2026-09-27


def apply_curve_outward_bias(desired_curvature: float, v_ego: float) -> float:
  # In a fast curve, command a slightly smaller curvature so the car runs wider and stops cutting the inside. Faded
  # out at low speed (tight curves there need the full turn-in or the car runs wide). Straights and small lane
  # corrections below the deadzone are untouched.
  frac = float(np.interp(v_ego, CURVE_OUTWARD_V_BP, CURVE_OUTWARD_FRAC_V))
  if frac == 0.0 or abs(desired_curvature) <= CURVE_OUTWARD_DEADZONE:
    return desired_curvature
  return desired_curvature * (1.0 - frac)   # frac < 0 scales the curvature UP (inward)

# Lateral jerk cap on the model's desired curvature (ported from tncr18). Softens the sharpest high-speed path
# corrections ("return to centre") while leaving steady centering untouched: capping |d(desired curvature)/dt| * v^2
# at 1.0 m/s^3 only touches the fastest few percent of frames, which then reach the same curvature about 0.1-0.2 s later.
# Bypassed during lane changes (controlsd has its own start-rate cap there) and when lateral control is inactive.
DESIRED_CURVATURE_MAX_LAT_JERK = 1.0  # m/s^3
DESIRED_CURVATURE_JERK_MIN_SPEED = 8.0  # m/s; below this the cap in curvature terms is so loose it barely acts
# Speed schedule (Altis 2026-09-27 21:14, route bf seg 11): after a torque-saturated tight turn at 41-47 km/h the model's
# desired curvature swung -22 -> +6 -> -4 e-3 (~6 m/s^3) and the car swayed twice; a 2.5 m/s^3 cap rounds that off while
# a normal turn-in (~1 m/s^3) is untouched. Off below 30 km/h (tight intersection turns need the full rate), 2.5 from
# 40 to 70 km/h, then the highway 1.0 from 80 km/h.
DESIRED_CURVATURE_JERK_FADE_BP = [8.3, 11.1, 19.4, 22.2]  # m/s (30, 40, 70, 80 km/h)
DESIRED_CURVATURE_JERK_FADE_V = [8.0, 2.5, 2.5, DESIRED_CURVATURE_MAX_LAT_JERK]


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
    desired_curvature = apply_curve_outward_bias(desired_curvature, CS.vEgo)
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
