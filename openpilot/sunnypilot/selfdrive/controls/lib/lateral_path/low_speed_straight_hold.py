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

# Low-speed straight hold. Owner 2026-10-20 (Altis, route 00000168 08:50:10-26): creeping straight at 1-8 km/h with no
# blinker and hands off, the wheel kept moving left and right. The model action's curvature hops around zero
# (+/-2.5e-3 1/m, the command -6.2..+7.8 deg at the wheel) while its own plan is straight (|k| <= 3e-3 over 4 s); none
# of our corrections act below ~29 km/h, and the same curvature noise is a large wheel angle at a crawl. Here, while
# the car is surely going straight (lateral active, no blinker, no lane change, hands off, wheel within ANGLE_MAX, the
# model plan's curvature within PLAN_K_MAX for the next PLAN_T s), the action's curvature has its small part
# (DEADBAND) removed and the rest low-passed over TAU: the wheel stays still. Full below V_FULL, faded out by V_OFF.
# Entering takes ENTER_T; as soon as any condition fails (a turn, a blinker) it lets go within RELEASE_T and the
# held value follows the model again at RELEASE_TAU, so turns are not delayed. Replay 08:50:10-26: command std
# 2.9 -> 1.0 deg; the turns at 08:49:06-16 and 08:50:40-47 untouched.
ENABLED = True
V_FULL = 10.0 / 3.6     # m/s
V_OFF = 20.0 / 3.6      # m/s
ANGLE_MAX = 10.0        # deg
PLAN_K_MAX = 1.0 / 150  # 1/m
PLAN_T = 4.0            # s
DEADBAND = 0.0015       # 1/m
TAU = 1.0               # s
RELEASE_TAU = 0.15      # s
ENTER_T = 0.5           # s
RELEASE_T = 0.3         # s


def plan_max_curvature(model_v2) -> float | None:
  try:
    t = np.asarray(model_v2.orientationRate.t)
    k = np.asarray(model_v2.orientationRate.z) / np.maximum(np.asarray(model_v2.velocity.x), 0.5)
  except Exception:
    return None
  if len(t) == 0 or len(k) != len(t):
    return None
  return float(np.max(np.abs(k[t <= PLAN_T])))


class LowSpeedStraightHold:
  def __init__(self, dt: float):
    self.dt = dt
    self.w = 0.0
    self.k = None

  def update(self, desired_curvature: float, model_v2, CS, active: bool, lane_changing: bool) -> float:
    v = CS.vEgo
    if not ENABLED or not active or v >= V_OFF:
      self.w, self.k = 0.0, None
      return desired_curvature
    blinker = getattr(CS, 'leftBlinker', False) or getattr(CS, 'rightBlinker', False)
    k_plan = plan_max_curvature(model_v2) if model_v2 is not None else None
    straight = (not lane_changing and not blinker and not CS.steeringPressed and abs(CS.steeringAngleDeg) < ANGLE_MAX
                and k_plan is not None and k_plan < PLAN_K_MAX)

    target = float(np.interp(v, [V_FULL, V_OFF], [1.0, 0.0])) if straight else 0.0
    if target > self.w:
      self.w = min(target, self.w + self.dt / ENTER_T)
    else:
      self.w = max(target, self.w - self.dt / RELEASE_T)

    if self.k is None:
      self.k = desired_curvature
    if straight:
      x = float(np.sign(desired_curvature) * max(abs(desired_curvature) - DEADBAND, 0.0))
      self.k += (x - self.k) * min(self.dt / TAU, 1.0)
    else:
      self.k += (desired_curvature - self.k) * min(self.dt / RELEASE_TAU, 1.0)
    return float((1.0 - self.w) * desired_curvature + self.w * self.k)
