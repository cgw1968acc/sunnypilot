import numpy as np
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
KP = 0.8  # = stock latcontrol_torque.KP (the highway end of the schedule is unchanged)

# 15-30 km/h gains halved for the Corolla Altis Hybrid (2026-09-27, route b7 seg 8): with the stock schedule
# (11.5 / 5.5 / 3.5 at 5 / 7.5 / 10 m/s) a wheel release mid-turn at 16 km/h turned into a full-authority limit cycle -
# torque saturated 71% of the time, P term +-4 on a 0.3 m/s^2 error, ~1.2 s period, steering +-40-56 deg.
KP_INTERP_TN = [250, 120, 65, 30, 6.0, 3.2, 2.0, 1.2, KP]  # 10/15 m/s 2.4/1.6 -> 2.0/1.2 (2026-09-27 21:13: 40-50 km/h curve-exit sway)


# Highway friction feed-forward scale (owner 2026-10-07 23:00, route 00000110, v1 torque controller: "lateral is stable,
# I like v1's centering sharpness, but I can still feel the corrections - make them a little smoother, without bringing
# the sway back"). On the 79-104 km/h straights the steering moved 0.25 deg in the 0.3-1 Hz band against 0.16 deg on
# V0 (route 00000109); the controller's own share is small (open-loop replays: <= 10%), the rest is the model loop.
# A low-pass on the path (tried 2026-10-05, removed in ce6d758ae1) adds lag in that loop and caused a 3.3 s weave, so
# this works on damping instead: in open-loop replays of 00000110 the friction feed-forward and the P term both act
# AGAINST the model-driven swing (friction x0.5: +11% 0.3-1 Hz torque, x0 +21%), and friction x1.3 lowered every
# measure (0.3-1 Hz -7%, 1-3 Hz -8%, torque rate -6%) while adding no lag. Applied on top of whatever friction torqued /
# the override set, above HWY_FRICTION_BP; centering, gains and the path are untouched.
HWY_FRICTION_BP = [60.0 / 3.6, 70.0 / 3.6]  # m/s
HWY_FRICTION_SCALE_V = [1.0, 1.3]


def get_highway_friction_scale(v_ego: float) -> float:
  return float(np.interp(v_ego, HWY_FRICTION_BP, HWY_FRICTION_SCALE_V))
