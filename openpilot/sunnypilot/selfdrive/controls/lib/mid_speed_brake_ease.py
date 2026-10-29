# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.constants import CV

# tnpb3 (Altis) 2026-10-25, owner after routes 00000173/00000174: "braking at 30-50 km/h a little lighter, the smallest
# change". At 30-50 km/h the request sent is the planner's own (10:38:18 -1.76 at 43 km/h behind a lead braking -1.6;
# 11:44:33-39 -0.84..-1.35 from 50 to 30 km/h); none of the controlsd layers acts there. So the planner's output target
# is eased 5 % there, faded in from 25 km/h and out by 55 km/h; below 25 km/h (the end of every stop) it is untouched.
# A hard request is not eased: the ease fades out from -1.8 to -2.2 m/s^2 (monotonic, no step). The MPC is seeded
# with this eased target, so it re-plans from what is actually asked for.
EASE = 0.05
EASE_V_BP = [25. * CV.KPH_TO_MS, 30. * CV.KPH_TO_MS, 50. * CV.KPH_TO_MS, 55. * CV.KPH_TO_MS]
EASE_V_W = [0., 1., 1., 0.]
EASE_A_BP = [-2.2, -1.8]  # m/s^2
EASE_A_W = [0., 1.]


def mid_speed_brake_ease(a_target: float, v_ego: float) -> float:
  if a_target >= 0.:
    return a_target
  w = float(np.interp(v_ego, EASE_V_BP, EASE_V_W)) * float(np.interp(a_target, EASE_A_BP, EASE_A_W))
  return a_target * (1. - EASE * w)
