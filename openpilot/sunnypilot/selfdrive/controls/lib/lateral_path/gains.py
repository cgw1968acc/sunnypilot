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
# tncr (Corolla Cross), owner 2026-10-15 night: the Cross keeps the stock schedule (tncr18/tncr19 b698798bcd: the
# Altis 6.0/3.2/2.0/1.2 left a left-right curve pair uncorrected on the Cross, 2026-10-05 22:19). Altis/tnpb3 keeps its own.
KP_INTERP_TN = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]  # Altis: [.., 6.0, 3.2, 2.0, 1.2, KP]  # 10/15 m/s 2.4/1.6 -> 2.0/1.2 (2026-09-27 21:13: 40-50 km/h curve-exit sway)


# Highway friction feed-forward scale (owner 2026-10-07 23:00, route 00000110, v1 torque controller: "lateral is stable,
# I like v1's centering sharpness, but I can still feel the corrections - make them a little smoother, without bringing
# the sway back"). On the 79-104 km/h straights the steering moved 0.25 deg in the 0.3-1 Hz band against 0.16 deg on
# V0 (route 00000109); the controller's own share is small (open-loop replays: <= 10%), the rest is the model loop.
# A low-pass on the path (tried 2026-10-05, removed in ce6d758ae1) adds lag in that loop and caused a 3.3 s weave, so
# this works on damping instead: in open-loop replays of 00000110 the friction feed-forward and the P term both act
# AGAINST the model-driven swing (friction x0.5: +11% 0.3-1 Hz torque, x0 +21%), and friction x1.3 lowered every
# measure (0.3-1 Hz -7%, 1-3 Hz -8%, torque rate -6%) while adding no lag. Applied on top of whatever friction torqued /
# the override set, above HWY_FRICTION_BP; centering, gains and the path are untouched.
HWY_FRICTION_BP = [60.0 / 3.6, 70.0 / 3.6, 100.0 / 3.6, 110.0 / 3.6]  # m/s
# 1.3 -> 1.4 (owner 2026-10-08, after the 00:07 "excellent" drive at 1.3 = tag altis-highway-lateral-baseline-1:
# "raise the highway friction a little, I'll try it"; open-loop 00000110 vs 1.3: 0.3-1 Hz -3%, 1-3 Hz -3%, rate -3%).
# 70 -> 100 km/h: 1.4 -> 1.5, flat above (owner 2026-10-13 night, after an "ideal" lateral drive: "can friction keep
# rising linearly above 70 km/h? - not too much, try it"; the smallest of the options shown, +7% at 100 km/h).
# tncr (Corolla Cross), owner 2026-10-15 night after the tncr19 lateral port ("this feels right for the Cross; add a
# x1.6 step for the highway friction and try"): one more point, 1.6 at 110 km/h, flat above; 100 km/h stays 1.5.
HWY_FRICTION_SCALE_V = [1.0, 1.4, 1.5, 1.6]


def get_highway_friction_scale(v_ego: float) -> float:
  return float(np.interp(v_ego, HWY_FRICTION_BP, HWY_FRICTION_SCALE_V))


# Low-speed feed-forward gain (tncr, owner 2026-10-25: mountain roads on ALKA at 30-60 km/h, "the model turns in early,
# then understeers and drifts from the inside toward the outside, sometimes onto the outer line - more steering
# ability"). 269 lateral-only curve events (backups 10-15/10-23/10-25, 33/40/49 km/h): the actual lateral accel lagged
# the request by 0.40 s (median) and reached only 0.83 -> 0.88 -> 0.91 -> 0.97 of it through the first four fifths of
# curves where the torque was NOT at its limit (75 of 104 clean events); 8 curves had the torque at the panda maximum
# all through the middle (actual 0.84 of the request, drifting 62 -> 26 cm toward the outside) - nothing can add there.
# The torque REQUEST moved faster than the panda rise limit (1500 per 1.0 s) for 40 % of curve time (p95 7 /s, PID
# jitter), so only the smooth feed-forward (the desired lateral accel) is scaled, not the PID. x1.12 puts the torque at
# its maximum for 31.9 % of curve time instead of 25.7 %.
LOW_FF_BP = [25.0 / 3.6, 30.0 / 3.6, 60.0 / 3.6, 70.0 / 3.6]  # m/s
LOW_FF_GAIN_V = [1.0, 1.12, 1.12, 1.0]


def get_low_speed_ff_gain(v_ego: float) -> float:
  return float(np.interp(v_ego, LOW_FF_BP, LOW_FF_GAIN_V))
