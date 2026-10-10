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

# Lead braking response by the drive mode (owner 2026-10-10: eco's stock braking is fine even when following close;
# std brakes a little sooner, sport a little more - "a little" = reacting about 0.3 s earlier). (MPC jerk-cost scale,
# lead-decel tau scale): a lower jerk cost lets the plan deepen the brake faster; a lower tau keeps the lead's measured
# decel in the prediction longer, so braking starts earlier. C3 dry run (real planner + MPC, RAV4, standard distance,
# 0.25 s lag), time to -0.5 / peak / closest gap:
#   50 km/h, lead -2.0 to stop:  eco 1.35 s -2.62 3.7 m | normal 1.15 s -2.52 4.8 m | sport 1.10 s -2.48 5.1 m
#   70 km/h, lead -1.0 for 3 s:  eco 2.60 s -0.69      | normal 2.25 s -0.76      | sport 2.05 s -0.79
#   40 km/h, lead -3.0 to stop:  eco 0.75 s -3.50 3.5 m | normal 0.60 s -3.21 4.5 m | sport 0.55 s -3.09 4.8 m
BRAKE_RESPONSE = {  # keyed by the accel personality name
  'eco':    (1.0, 1.0),
  'normal': (0.75, 0.65),
  'sport':  (0.65, 0.55),
}


class AccelPersonalityController:
  def __init__(self):
    self.personality = 'eco'  # until the car reports a mode: the branch's own profile

  def update(self, sm) -> None:
    if 'carStateSP' in sm.data and sm.recv_frame['carStateSP'] > 0:
      self.personality = str(sm['carStateSP'].accelPersonality)

  def apply_brake_response(self, mpc) -> None:
    mpc.jerk_scale, mpc.lead_tau_scale = BRAKE_RESPONSE.get(self.personality, BRAKE_RESPONSE['eco'])

  def max_accel(self, v_ego: float, stock_max: float) -> float:
    profile = MAX_ACCEL_PROFILES.get(self.personality)
    if profile is None:  # eco (or unknown)
      return float(stock_max)
    return float(np.interp(v_ego, MAX_ACCEL_BP, profile))
