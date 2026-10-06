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
