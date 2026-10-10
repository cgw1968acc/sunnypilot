"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

# TN: acceleration personality picked by the car's drive mode switch (carStateSP.accelPersonality, set in the Toyota
# carstate from GEAR_PACKET). Owner 2026-10-10: eco = the owner's own single profile of this branch (the stock-file
# A_CRUISE_MAX_VALS [1.6, 0.75, 0.28, 0.08] at [0, 10, 25, 40] m/s, passed in as stock_max), normal and sport = the
# owner's tnc3C tables (vibe_personality, 2026-04-14).
MAX_ACCEL_BP = [0., 3., 5., 8., 12., 18., 24., 32., 42.]  # m/s
MAX_ACCEL_PROFILES = {  # keyed by the enum name: a capnp enum read from a message does not hash like the schema constant
  'normal': [1.99, 1.80, 1.55, 0.94, 0.74, 0.60, 0.35, 0.22, 0.09],
  'sport':  [2.00, 1.95, 1.80, 1.46, 1.20, 0.88, 0.62, 0.31, 0.11],
}


class AccelPersonalityController:
  def __init__(self):
    self.personality = 'eco'  # until the car reports a mode: the branch's own profile

  def update(self, sm) -> None:
    if 'carStateSP' in sm.data and sm.recv_frame['carStateSP'] > 0:
      self.personality = str(sm['carStateSP'].accelPersonality)

  def max_accel(self, v_ego: float, stock_max: float) -> float:
    profile = MAX_ACCEL_PROFILES.get(self.personality)
    if profile is None:  # eco (or unknown)
      return float(stock_max)
    return float(np.interp(v_ego, MAX_ACCEL_BP, profile))
