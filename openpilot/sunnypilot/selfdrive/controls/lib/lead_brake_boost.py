"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.realtime import DT_CTRL

# Lead hard-brake boost. Owner 2026-10-13 night, route 00000157 (device clock 23:29:36-47, Aggressive, 55 km/h): the lead
# braked at -3.6 m/s^2 to a stop; we peaked at -3.6, but below ~15 km/h the plan already eased (-1.47 at 9.9 km/h with
# the stopped lead 6.5 m ahead, -1.17 at 6.7 km/h / 4.0 m, -0.75 at 3.8 km/h / 2.4 m) and the car delivered less
# (-1.01 / -0.99 / -0.86); at 1.8 km/h it crept on the hybrid's creep torque (-0.28 -> 0) from 1.5 to 1.0 m and the
# driver had to stop it at 0.8 m: "the hard stop lacked force at the very end, it almost hit the car in front".
# Owner's rule: "do not change the 2-0 km/h end force; what needs more is 30 to 3 km/h; multiply by 1.2 to try, too heavy
# is uncomfortable". So once the lead has braked hard (aLeadK <= TRIGGER), every braking request between 3 and 30 km/h is
# multiplied by GAIN until the car stops or the lead drives off. On 00000157 the car travelled 15.35 m from 30 to 3 km/h;
# 1.2x the decel over the same profile is ~2.6 m shorter -> a stop ~3.4 m behind the lead instead of 0.8 m.
# No bite (owner top rule): the factor ramps in at GAIN_RATE, it only scales a request that is already braking, and the
# Toyota brake-onset shaping downstream is unchanged.
TRIGGER_A = -2.0      # m/s^2: lead accel (filtered) that marks a hard brake
RELEASE_A = 0.3       # m/s^2: the lead accelerating faster than this while not slower than us releases it
GAIN = 1.2
GAIN_RATE = 0.5       # 1/s: 1.0 -> 1.2 in 0.4 s, and back
BOOST_BP = [2.0 / 3.6, 3.0 / 3.6, 30.0 / 3.6, 33.0 / 3.6]  # m/s
BOOST_W = [0.0, 1.0, 1.0, 0.0]


class LeadBrakeBoost:
  def __init__(self):
    self.latched = False
    self.gain = 1.0

  def update(self, accel: float, v_ego: float, standstill: bool, lead, long_active: bool, accel_min: float) -> float:
    present = bool(lead.present)
    if not long_active or standstill or not present:
      self.latched = False
    elif lead.aLeadK <= TRIGGER_A:
      self.latched = True
    elif lead.aLeadK > RELEASE_A and lead.vLead >= v_ego:
      self.latched = False

    target = 1.0 + (GAIN - 1.0) * float(np.interp(v_ego, BOOST_BP, BOOST_W)) if self.latched else 1.0
    step = GAIN_RATE * DT_CTRL
    self.gain = float(np.clip(target, self.gain - step, self.gain + step))
    if accel < 0.0 and self.gain > 1.0:
      return max(accel * self.gain, accel_min)
    return accel
