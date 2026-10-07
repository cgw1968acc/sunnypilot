"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter

# Lane centering feedback, ported from tncr18 (Corolla Cross) on 2026-09-30 for the Altis: rlog 2026-09-27 (6 routes,
# Macrostiff) above 100 km/h the car sat 0.10 m left of the lane centre on average with a 0.23 m std (22% of the time more
# than 0.3 m off; 80-100 km/h: 4%), while the torque loop tracked the model's curvature closely (0.73 vs 0.84 e-3 std) -
# the wandering is the model path drifting, so close the loop on the lane lines.
# Original tncr18 note (route 4d seg 9): in a 70 m-radius curve at 47 km/h the car sat
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
LANE_CENTER_DEADBAND = 0.02        # m; no position correction inside this (0.015 -> 0.02 2026-10-08, see LAT_SPEED_DEADBAND) (0.03 -> 0.015, 2026-09-30: the driver felt the
                                   # ... -> 0.0075 2026-10-04: route 000000f2 10:34 at 82-88 km/h he felt the corrections again
                                   # and asked for a smaller dead band once more. Note: the offset there wandered -20..+15 cm
                                   # with a 4-6 s rhythm, far outside any dead band, so the next lever is the gain, not this.
                                   # corrections at 93 km/h as a regular drift-correct rhythm (~4 s, +-1.3 deg of steering) and
                                   # asked for smaller, more continuous ones; a narrower dead band starts them earlier and smaller)
# 2026-10-06: back to 0.015 (from 0.0075) with the highway smoothing below; recomputed on the highway logs the smaller dead
# band added only ~10% more centering correction, and the driver felt more side-to-side motion on tnpb2.
# 2026-10-08 (owner, after friction x1.3/x1.4 made the highway steadier: "with the higher friction, the dead band and
# the correction frequency can be relaxed a little"): open-loop over route 00000110 (191 s > 75 km/h) the centering
# correction is a ~4.1 s rhythm, std 0.044 m/s^2, above 0.05 m/s^2 33% of the time - and most of that activity is the
# DAMPING (lateral speed) term, which had no dead band: widening the position dead band alone (1.5 -> 3-4 cm) changed it
# by <= 8%. A small dead band on the lateral speed as well: 2.0 cm / 1 cm/s -> std 0.036 (-18%), 0.3-1 Hz -17%, rate
# -19%, active 19% of the time (2.5 / 1.5: -27%; 3 / 3: -48%). First step 2.0 cm / 1 cm/s.
LANE_CENTER_LAT_SPEED_DEADBAND = 0.01  # m/s; no damping correction for a lateral drift slower than this (was none)
LANE_CENTER_KP = 0.35              # m/s^2 of lateral accel per metre of offset
LANE_CENTER_KD = 1.20              # m/s^2 per m/s of lateral speed toward/away from the centre (damping, ~critical)
LANE_CENTER_KI = 0.05              # m/s^2 per metre per second: slowly takes over what the model keeps pulling
LANE_CENTER_I_LIMIT = 0.45         # m/s^2; cap on the integrated part
# 2026-10-06 (route 00000109 23:27-23:34, ~103 km/h): the car sat LEFT of the lane centre the whole stretch (straight
# -10 cm, left curves -20..-35, right curves -13..-24 cm) with the centering saturated near 0.35-0.43 m/s^2 against the
# model's pull; the integral cap 0.25 -> 0.45 m/s^2 lets it finish the job. The total stays capped by MAX_LAT_ACCEL.
# The centering has no upper speed limit (full from 80 km/h up, e.g. 140 km/h).
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
    lateral_speed -= float(np.clip(lateral_speed, -LANE_CENTER_LAT_SPEED_DEADBAND, LANE_CENTER_LAT_SPEED_DEADBAND))

    self.integral = float(np.clip(self.integral + LANE_CENTER_KI * error * self.dt, -LANE_CENTER_I_LIMIT, LANE_CENTER_I_LIMIT))
    lat_accel = LANE_CENTER_KP * error + LANE_CENTER_KD * lateral_speed + self.integral
    lat_accel = float(np.clip(lat_accel, -LANE_CENTER_MAX_LAT_ACCEL, LANE_CENTER_MAX_LAT_ACCEL))
    target = float(np.clip(lat_accel / max(v_ego, LANE_CENTER_MIN_SPEED) ** 2, -LANE_CENTER_MAX_CURV, LANE_CENTER_MAX_CURV))
    target *= float(np.interp(v_ego, LANE_CENTER_FADE_BP, LANE_CENTER_FADE_V))
    self.correction = float(np.clip(target, self.correction - step, self.correction + step))
    return self.correction


# Low-speed lane trim (below the highway-speed centering), ported from tncr18 on 2026-09-30 (Altis driver: "sometimes
# the car is not centred at low speed either"). Original note - Route 58 seg 4 and route 4d segs 9/11 (Corolla Cross,
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
