"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.realtime import DT_CTRL

# Stop-gap governor. Owner 2026-10-10 night: "the new end is better, but the car still rolls in a little too close
# before it stops - control it so the car stops 3.5 to 3.75 m behind the lead". Routes 0000011e:
#   21:04  the plan reached 1 km/h 3.7 m behind; then, at the -0.30 end request, the car ACCELERATED again
#          (aEgo +0.15) and crept 1.0 km/h for 1.5 s -> stopped at 3.0 m
#   21:09  below 8 km/h the plan braked less than the gap needed (-0.85..-0.99 where -1.0..-1.7 was needed to stop at
#          3.6 m); 1 km/h at 2.7 m, then the same creep for 4 s (BRAKE 0xA6 fell to 40 N) -> 1.4 m
#   21:58  the lead braked late; 1 km/h at 2.1 m, then crept 2.0 -> 1.1 m until the driver braked
# Two parts, both only ever FIRMER than what the stopping controller asks (never lighter, so the end table's feel
# stays wherever the gap is on track):
# 1) gap floor, below GAP_V_MAX behind a lead: the decel that stops the car GAP_TARGET behind where the lead will stop,
#    a = -v^2 / (2 * (dRel - GAP_TARGET + vLead^2 / (2 * LEAD_DECEL))). It enters from the current request and moves at
#    GAP_JERK, capped at GAP_MAX_DECEL, so it never steps.
# 2) creep stop, below CREEP_V_MAX at the end of a stop: once the speed rises CREEP_RISE above its lowest value (the
#    hybrid's creep torque beating the light end request), the request goes to CREEP_ACCEL at CREEP_JERK until the
#    wheels stop; the hold then continues from there.
GAP_TARGET = 3.6  # m, middle of the owner's 3.5-3.75 m
GAP_V_MAX = 15.0 / 3.6  # m/s
GAP_LEAD_V_MAX = 3.0  # m/s: the lead is stopping or crawling (a moving lead is the planner's business)
LEAD_DECEL = 2.0  # m/s^2: assumed braking of a lead that is still rolling
GAP_S_MIN = 0.4  # m: distance floor in the formula (at or inside the target: the cap applies)
GAP_MAX_DECEL = -1.8  # m/s^2
GAP_JERK = 1.5  # m/s^3, both ways
CREEP_V_MAX = 3.0 / 3.6  # m/s
CREEP_RISE = 0.2 / 3.6  # m/s
CREEP_ACCEL = -0.7  # m/s^2 (~1300 N+; 21:58 still crept at 1000-1300 N)
CREEP_JERK = 3.0  # m/s^3: -0.30 -> -0.70 in ~0.13 s


class StopGapGovernor:
  def __init__(self):
    self.floor = 0.0  # 0 = inactive
    self.active = False
    self.v_min = None
    self.creep_stop = False

  def reset(self):
    self.floor, self.active, self.v_min, self.creep_stop = 0.0, False, None, False

  def update(self, out, v_ego, standstill, a_target, braking, lead_d=None, lead_v=None):
    """out: the stopping controller's request; braking: the car is braking to a stop (end window / stopping state /
    a plan below -0.25). Returns the request, firmer where the gap needs it."""
    if standstill or a_target > 0.0:
      self.reset()
      return out

    # 2) creep stop
    if braking and v_ego < CREEP_V_MAX:
      self.v_min = v_ego if self.v_min is None else min(self.v_min, v_ego)
      if v_ego > self.v_min + CREEP_RISE:
        self.creep_stop = True
    elif v_ego >= CREEP_V_MAX:
      self.v_min, self.creep_stop = None, False
    if self.creep_stop:
      self.floor = max(CREEP_ACCEL, min(self.floor if self.active else out, out) - CREEP_JERK * DT_CTRL)
      self.active = True
      return min(out, self.floor)

    # 1) gap floor
    want = None
    if braking and lead_d is not None and v_ego < GAP_V_MAX and lead_v is not None and lead_v < GAP_LEAD_V_MAX:
      lead_stop = max(lead_v, 0.0) ** 2 / (2 * LEAD_DECEL)
      s = max(lead_d - GAP_TARGET + lead_stop, GAP_S_MIN)
      want = max(-v_ego ** 2 / (2 * s), GAP_MAX_DECEL)
    if want is not None and (self.active or want < out):
      if not self.active:
        self.floor, self.active = out, True
      step = GAP_JERK * DT_CTRL
      self.floor = float(np.clip(want, self.floor - step, self.floor + step))
    elif self.active:
      self.floor += GAP_JERK * DT_CTRL  # lead gone / out of range: let go at the same rate
    if self.active and self.floor >= out:
      self.active, self.floor = False, 0.0
    return min(out, self.floor) if self.active else out
