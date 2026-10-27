"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.lane_centering import LaneCentering, LANE_CENTER_FILTER_TAU

# Low-speed lane position feedback (owner 2026-10-12 night, route 00000140, v1 below 55 km/h):
# - 23:36 left curve into right curve at 47-51 km/h: "it went right before it was back in the centre". The car ran
#   0.7-0.8 m inside the left curve, was already 0.4 m right at the S transition and 1.0-1.1 m inside the right curve
#   (the outer line at prob 0.38-0.5). Over all 35-65 km/h curves of the route: median 0.76 m toward the inside.
# - 23:41:40-46, after an intersection: "it stays between two lanes, I want it to pick the lane it is more in and go
#   there quicker". The car came out ~1 m right with the dashed line under its right half; the model planned straight
#   on and the car needed ~4 s to recentre.
# LaneTrimLowSpeed (integral only, 1 s engage delay, 0.25 m/s^2 cap) gave -0.13..-0.21 m/s^2 late in the curve and ~0
# after the intersection. This replaces it with a PD on the measured lane position, like the highway LaneCentering:
# outside a DEADBAND around the lane centre, lateral accel = KP * error + KD * lateral speed, capped and jerk limited.
# The lane is the one the car's CENTRE is in (the near-left / near-right line pair), so a car straddling a line goes
# to the lane it is more in. Open loop on route 00000140: -0.40..-0.54 m/s^2 at 23:35:57-59 and -0.30 m/s^2 at 23:41:41,
# from the first frame the lines are back.
# Off with a blinker, during a lane change, while the driver steers and when lateral is inactive; the lines briefly lost
# (an intersection, a weak outer line) hold the correction for LOST_HOLD_T, then it eases to zero. Fades out 70 -> 80
# km/h, where the highway LaneCentering fades in.
# tncr (Corolla Cross), owner 2026-10-27, route 000000ba 22:31 (the same left-into-right curve pair, 63 km/h, lane
# 4.2 m): the car ran 0.9 m inside the left curve and then 1.14 m inside (right) in the right curve, its right side on
# the line; the highway centering only starts at 70 km/h and the older trim (kept below) is capped at 0.25 m/s^2.
# Owner chose: add this PD at 50-70 km/h, ONLY in curves and only with both lines clear. So on the Cross it runs on
# top of the trim (which is unchanged), faded in 45 -> 50 km/h, weighted by the road curvature (LOW_CENTER_CURVE_*,
# held through an S-bend's zero crossing by a slow decay) and only with both near lines >= LOW_CENTER_MIN_PROB.
LOW_CENTER_ENABLED = True
LOW_CENTER_MIN_SPEED = 8.0          # m/s (~29 km/h; the shared lane measurement starts here too)
LOW_CENTER_FADE_BP = [45.0 / 3.6, 50.0 / 3.6, 70.0 / 3.6, 80.0 / 3.6]  # m/s: tncr 50-70 km/h only
LOW_CENTER_FADE_V = [0.0, 1.0, 1.0, 0.0]
LOW_CENTER_CURVE_BP = [1.0e-3, 2.0e-3]  # 1/m |curvature|: none on a straight (R > 1000 m), full from R 500 m
LOW_CENTER_CURVE_DECAY = 2.0e-3     # 1/m per s: the held curvature falls this fast (an S-bend crosses zero in ~1-2 s)
LOW_CENTER_MIN_PROB = 0.6           # both near lines at least this
LOW_CENTER_DEADBAND = 0.25          # m: no correction this close to the lane centre
LOW_CENTER_KP = 0.6                 # m/s^2 per metre beyond the dead band
LOW_CENTER_KD = 0.9                 # m/s^2 per m/s of lateral speed (damping; only outside the dead band)
LOW_CENTER_MAX_LAT_ACCEL = 0.8      # m/s^2
LOW_CENTER_JERK = 0.6               # m/s^3: rate limit on the correction
LOW_CENTER_LOST_HOLD_T = 1.0        # s


class LaneCenteringLowSpeed:
  def __init__(self, dt: float, centering: LaneCentering):
    self.dt = dt
    self.centering = centering  # shares its lane-geometry measurement (made once per frame in its update)
    self.offset_filter = FirstOrderFilter(0.0, LANE_CENTER_FILTER_TAU, dt)
    self.heading_filter = FirstOrderFilter(0.0, LANE_CENTER_FILTER_TAU, dt)
    self.lat_accel = 0.0  # target before the rate limit
    self.correction = 0.0
    self.lost_t = 0.0
    self.valid = False
    self.curve_k = 0.0  # held |curvature| of the road

  def update(self, model_v2, v_ego: float, active: bool, steering_pressed: bool, blinker: bool = False,
             lane_changing: bool = False, curvature: float = 0.0) -> float:
    """Curvature to ADD to the desired curvature (positive = right). curvature: the model's desired curvature."""
    self.curve_k = max(abs(curvature), self.curve_k - LOW_CENTER_CURVE_DECAY * self.dt)
    usable = active and not steering_pressed and not blinker and not lane_changing and v_ego > LOW_CENTER_MIN_SPEED
    meas = self.centering.last_meas if usable else None
    if meas is not None and model_v2 is not None and len(model_v2.laneLineProbs) >= 4 and \
       min(model_v2.laneLineProbs[1], model_v2.laneLineProbs[2]) < LOW_CENTER_MIN_PROB:
      meas = None  # tncr: both lines must be clear
    self.valid = meas is not None
    if meas is not None:
      self.lost_t = 0.0
      offset = self.offset_filter.update(meas[0])  # + = car LEFT of the centre -> positive (right) correction
      heading = self.heading_filter.update(meas[1])
      if abs(offset) > LOW_CENTER_DEADBAND:
        error = offset - math.copysign(LOW_CENTER_DEADBAND, offset)
        lateral_speed = v_ego * math.sin(heading)  # + = drifting LEFT
        lat = LOW_CENTER_KP * error + LOW_CENTER_KD * lateral_speed
      else:
        lat = 0.0
      self.lat_accel = float(np.clip(lat, -LOW_CENTER_MAX_LAT_ACCEL, LOW_CENTER_MAX_LAT_ACCEL))
    elif usable:
      self.lost_t += self.dt  # lines briefly lost: hold, then let go
      if self.lost_t > LOW_CENTER_LOST_HOLD_T:
        self.lat_accel = 0.0
    else:
      self.lost_t = 0.0
      self.lat_accel = 0.0

    v2 = max(v_ego, LOW_CENTER_MIN_SPEED) ** 2
    target = self.lat_accel / v2 * float(np.interp(v_ego, LOW_CENTER_FADE_BP, LOW_CENTER_FADE_V)) * \
      float(np.interp(self.curve_k, LOW_CENTER_CURVE_BP, [0.0, 1.0]))
    step = LOW_CENTER_JERK / v2 * self.dt
    self.correction = float(np.clip(target, self.correction - step, self.correction + step))
    return self.correction
