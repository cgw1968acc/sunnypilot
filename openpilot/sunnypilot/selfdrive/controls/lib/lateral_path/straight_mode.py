"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.highway_plan_curvature import curve_weight, plan_curvature

# Straight mode (tncr, Corolla Cross; owner 2026-10-17 night after route 00000086: "on straights I want the correction
# frequency at the highest, to try" - small corrections before the car has drifted far). The model side already has no
# smoothing (LAT_SMOOTH_SECONDS 0) and the loop runs at 100 Hz; what slows the corrections on a straight are our own
# limits: the desired-curvature jerk cap (1.0 m/s^3 above 80 km/h) and the highway lane centering (3 cm dead band, 0.5 s
# filter on the measured offset, 0.5 m/s^3 rate limit). On a straight above 70 km/h they go to their fastest:
# jerk cap 8 m/s^3 (its own low-speed value, i.e. effectively off), centering dead band 1.5 cm (the Altis value),
# filter 0.25 s, rate 1.0 m/s^3. Straight = the road the model plans 0.3-3 s ahead needs less than ~0.1 m/s^2 (+1 deg of wheel angle) of
# lateral accel (same split as highway_plan_curvature: route 00000084 straights < 0.10, curves > 0.2 m/s^2; since
# 10-17 night widened by 1 deg of wheel angle, see highway_plan_curvature.curve_weight). The weight
# moves at RATE, so entering / leaving a curve blends over 1 s. Not simulated closed loop - road test.
ENABLED = True
V_BP = [65.0 / 3.6, 70.0 / 3.6]  # m/s
T_CURVE = 3.0                    # s
RATE = 1.0                       # 1/s
STRAIGHT_CURV_JERK = 8.0         # m/s^3
STRAIGHT_CENTER_DEADBAND = 0.015  # m
STRAIGHT_CENTER_TAU = 0.25       # s
STRAIGHT_CENTER_JERK = 1.0       # m/s^3


class StraightMode:
  def __init__(self, dt: float):
    self.dt = dt
    self.w = 0.0

  def update(self, model_v2, v_ego: float, active: bool, lane_changing: bool) -> float:
    """0 = normal, 1 = straight (fastest corrections)"""
    target = 0.0
    if ENABLED and active and not lane_changing and model_v2 is not None:
      k_road = plan_curvature(model_v2, T_CURVE)
      if k_road is not None:
        target = (1.0 - curve_weight(k_road, v_ego)) * float(np.interp(v_ego, V_BP, [0.0, 1.0]))
    self.w = float(np.clip(target, self.w - RATE * self.dt, self.w + RATE * self.dt))
    return self.w


def blend(normal: float, straight: float, w: float) -> float:
  return normal + (straight - normal) * w
