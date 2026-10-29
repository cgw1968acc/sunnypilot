# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.realtime import DT_CTRL

# Highway plan curvature (tncr, Corolla Cross; owner 2026-10-17, route 00000084 21:40:32, 96 km/h, one bookmark: "above
# ~90 km/h the corrections should be softer"). On a long gentle curve the model's action.desiredCurvature swung
# 0.24 <-> 0.9 /km every 3-4 s while the torque controller followed it closely (P and I tiny, the feed-forward carried
# it) - the swing is the model's next-instant curvature, not a control error, so faster or firmer correction would only
# follow it harder. The model's own planned path is steadier: the curvature it plans over T_LO-T_HI ahead
# (orientationRate.z / velocity.x, averaged) tracks the action with no lag (correlation 0.995 over 113 s > 90 km/h)
# but carries less of the swing. It keeps only ~0.92 of the slow curve, so the slow part of the action is put back:
# out = plan + low-pass(action - plan, BIAS_TAU). Route 00000084 > 90 km/h: 0.3-1 Hz swing -39%, 0.1-0.3 Hz -21%,
# 1-3 Hz -27%, slow curves (< 0.1 Hz) 1.00, lag 0. Not a low-pass of the curvature itself (the 0.5 s smoother of 10-08
# weaved at 80 km/h because it lagged). Faded in from V_LO to V_HI; faded out at W_RATE during a lane change, without
# lateral control or with a maneuver plan.
# Owner 2026-10-17 night: "lower it to above 70 km/h" -> faded in 65 -> 70 km/h (was 85 -> 90). Same replay, 70-80 km/h
# (84 s): 0.3-1 Hz -47%, 0.1-0.3 Hz -32%, slow curves -4%, largest difference 0.08 m/s^2; 80-90 km/h (69 s): -39% / -25%
# / -2%, 0.13 m/s^2.
# Owner 2026-10-17 night road test: "on straights the averaged curvature sways slowly left and right; on a straight go
# back to the original". So it is used only in a curve: the lateral accel of the road the model plans T_LO..T_CURVE
# ahead must reach CURVE_A_HI for the full blend and is zero below CURVE_A_LO (route 00000084 > 70 km/h: straights
# below 0.10 m/s^2, curves above 0.2 - the 21:40 curve 0.22..0.45, median 0.38; 42% of the time below 0.10). The weight
# still moves at W_RATE, so entering and leaving a curve is a 1 s blend, not a step.
ENABLED = True
V_LO = 65.0 / 3.6   # m/s
V_HI = 70.0 / 3.6   # m/s
T_LO = 0.3          # s
T_HI = 1.5          # s
BIAS_TAU = 2.0      # s
W_RATE = 1.0        # 1/s: blend weight change rate
T_CURVE = 3.0       # s
CURVE_A_LO = 0.10   # m/s^2
CURVE_A_HI = 0.20   # m/s^2
# Owner 2026-10-17 night: "widen the straight-mode angle by one degree" -> both edges move by 1 deg of steering wheel
# angle on top (Corolla Cross: wheelbase 2.67 m x steer ratio 13.9 = 37.1 m per rad of wheel angle); then "cancel, add
# 0.5 deg only": straight below ~0.8 deg wheel at 100 km/h (was 0.3), ~0.7 deg at 120, ~1.1 at 70. Shared with
# straight_mode.py so the two never overlap.
STRAIGHT_EXTRA_CURV = float(np.radians(0.5)) / (2.67 * 13.9)  # 1/m (owner, same night: "cancel, +0.5 deg only")
# Gentle curves only (tncr, owner 2026-10-18 after route 0000008b 23:44:43-51, 87 km/h exit-ramp S-bend, "a small flaw"):
# in a sharp curve (model up to -2.9 m/s^2) the plan average turned in 16% short (-2.45 for -2.92) and the 2 s bias
# carried that into the exit (-1.30 sent for -0.94), 0.52 m/s^2 apart at most; the car drifted ~0.5 m in the lane.
# The plan average is built for gentle curves (route 00000084's 21:40 curve <= 0.49 m/s^2): it fades out between
# SHARP_A_LO and SHARP_A_HI of road lateral accel, back to the model's own action. Route 0000008b > 70 km/h: 22% of the
# time above 0.8; route 00000084 > 70 km/h: never.
SHARP_A_LO = 0.8   # m/s^2
SHARP_A_HI = 1.2   # m/s^2


def plan_curvature(model_v2, t_hi: float = T_HI) -> float | None:
  t = np.asarray(model_v2.orientationRate.t, dtype=float)
  yaw_rate = np.asarray(model_v2.orientationRate.z, dtype=float)
  vx = np.asarray(model_v2.velocity.x, dtype=float)
  if len(t) == 0 or len(t) != len(yaw_rate) or len(t) != len(vx):
    return None
  window = (t >= T_LO) & (t <= t_hi)
  if not window.any():
    return None
  return float(np.mean(yaw_rate[window] / np.maximum(vx[window], 1.0)))


def curve_weight(k_road: float, v_ego: float) -> float:
  """0 on a straight, 1 in a curve, from the road curvature the model plans 0.3-T_CURVE s ahead"""
  extra = STRAIGHT_EXTRA_CURV * v_ego ** 2
  return float(np.interp(abs(k_road) * v_ego ** 2, [CURVE_A_LO + extra, CURVE_A_HI + extra], [0.0, 1.0]))


class HighwayPlanCurvature:
  def __init__(self, dt: float = DT_CTRL):
    self.dt = dt
    self.reset()

  def reset(self) -> None:
    self.bias: float | None = None
    self.w = 0.0

  def update(self, desired_curvature: float, model_v2, v_ego: float, active: bool, lane_change_off: bool) -> float:
    k_plan = plan_curvature(model_v2) if ENABLED and active else None
    if k_plan is None:
      self.reset()
      return desired_curvature
    diff = desired_curvature - k_plan
    self.bias = diff if self.bias is None else self.bias + (diff - self.bias) * self.dt / BIAS_TAU
    k_road = plan_curvature(model_v2, T_CURVE)
    curve = curve_weight(k_road, v_ego)  # 0 on a straight
    curve *= float(np.interp(abs(k_road) * v_ego ** 2, [SHARP_A_LO, SHARP_A_HI], [1.0, 0.0]))  # gentle curves only
    w_target = float(np.interp(v_ego, [V_LO, V_HI], [0.0, 1.0])) * curve if lane_change_off else 0.0
    self.w = float(np.clip(w_target, self.w - W_RATE * self.dt, self.w + W_RATE * self.dt))
    if self.w <= 0.0:
      return desired_curvature
    return desired_curvature + self.w * (k_plan + self.bias - desired_curvature)
