"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import custom
import numpy as np
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params

AccelPersonality = custom.LongitudinalPlanSP.AccelerationPersonality
ACCEL_PERSONALITY_OPTIONS = [AccelPersonality.eco, AccelPersonality.normal, AccelPersonality.sport]

# Acceleration Profiles
AccelPersonality = custom.LongitudinalPlanSP.AccelerationPersonality
ACCEL_PERSONALITY_OPTIONS = [AccelPersonality.eco, AccelPersonality.normal, AccelPersonality.sport]

# Acceleration Profiles
MAX_ACCEL_PROFILES = {
  AccelPersonality.eco:       [1.85, 1.80, 1.54, 0.93, 0.68, 0.57, 0.33, 0.10, 0.08, 0.07],
  AccelPersonality.normal:    [2.00, 1.95, 1.60, 1.06, 0.77, 0.64, 0.37, 0.11, 0.09, 0.08],
  AccelPersonality.sport:     [2.00, 2.00, 1.85, 1.48, 1.10, 0.88, 0.52, 0.14, 0.11, 0.09],
}
MAX_ACCEL_BREAKPOINTS =       [0.0,  3.0,  5.0,  8.0,  12.0, 18.0, 24.0, 32.0, 42.0, 55.0]

MIN_ACCEL_PROFILES = {
  #AccelPersonality.eco:       [-0.0021, -0.2400, -0.0060, -0.0070, -0.16, -0.30, -0.76], #corolla only
  #AccelPersonality.normal:    [-0.0022, -0.2600, -0.0061, -0.0071, -0.17, -0.32, -0.81], #corolla only
  #AccelPersonality.sport:     [-0.7700, -0.2800, -0.0062, -0.0072, -0.18, -0.34, -0.86], #corolla only
  AccelPersonality.eco:       [-0.0021, -0.2400, -0.0060, -0.0070, -0.32, -0.60, -1.00], #cross
  AccelPersonality.normal:    [-0.0022, -0.2600, -0.0061, -0.0071, -0.34, -0.62, -1.10], #cross
  AccelPersonality.sport:     [-0.7700, -0.2800, -0.0062, -0.0072, -0.36, -0.64, -1.20], #cross
}
#MIN_ACCEL_BREAKPOINTS =       [2.0,     40.0]
MIN_ACCEL_BREAKPOINTS =       [2.0,     4.4,    5.5,     10.0,  16.0,  22.0,  40.0]

ACCEL_ALPHA_BASE = 0.30   # responsive for small corrections
ACCEL_ALPHA_MAX = 0.85    # smooth for big transitions
ACCEL_ALPHA_SCALE = 0.8   # How fast alpha grows with error

DECEL_ALPHA_BASE = 0.35   # smooth even for small changes
DECEL_ALPHA_MIN = 0.15    # responsive
DECEL_ALPHA_SCALE = -1.5  # decel gets more responsive as error grows

MAX_DECEL_INCREASE_RATE = 0.5  # slow brake onset for coast feel (m/s² per second)
MAX_DECEL_DECREASE_RATE = 0.8  # faster brake release (m/s² per second)


class AccelPersonalityController:
  def __init__(self):
    self.params = Params()
    self.frame = 0
    self.last_max_accel = 2.0
    self.last_min_accel = -0.01
    self.first_run = True
    self._accel_personality = self.params.get('AccelPersonality') or AccelPersonality.normal
    self._enabled = self.params.get_bool('AccelPersonalityEnabled')

  def update(self, sm=None):
    self.frame += 1
    if self.frame % int(1.0 / DT_MDL) == 0:
      self._accel_personality = self.params.get('AccelPersonality') or AccelPersonality.normal
      self._enabled = self.params.get_bool('AccelPersonalityEnabled')

  @property
  def accel_personality(self) -> int:
    return self._accel_personality

  def get_accel_personality(self) -> int:
    return int(self._accel_personality)

  def set_accel_personality(self, personality: int):
    if personality in ACCEL_PERSONALITY_OPTIONS:
      self._accel_personality = personality
      self.params.put('AccelPersonality', personality)

  def cycle_accel_personality(self) -> int:
    current = self._accel_personality
    next_personality = ACCEL_PERSONALITY_OPTIONS[(ACCEL_PERSONALITY_OPTIONS.index(current) + 1) % len(ACCEL_PERSONALITY_OPTIONS)]
    self.set_accel_personality(next_personality)
    return int(next_personality)

  @staticmethod
  def _adaptive_alpha(current: float, target: float, base: float, limit: float, scale: float) -> float:
    error = abs(target - current)
    if scale < 0:
      return max(limit, base + error * scale)
    return min(limit, base + error * scale)

  def get_accel_limits(self, v_ego: float) -> tuple[float, float]:
    v_ego = max(0.0, v_ego)
    target_max = np.interp(v_ego, MAX_ACCEL_BREAKPOINTS, MAX_ACCEL_PROFILES[self.accel_personality])
    target_min = np.interp(v_ego, MIN_ACCEL_BREAKPOINTS, MIN_ACCEL_PROFILES[self.accel_personality])

    if self.first_run:
      self.last_max_accel, self.last_min_accel = target_max, target_min
      self.first_run = False
      return float(target_min), float(target_max)

    # Adaptive accel smoothing
    accel_alpha = self._adaptive_alpha(self.last_max_accel, target_max, ACCEL_ALPHA_BASE, ACCEL_ALPHA_MAX, ACCEL_ALPHA_SCALE)
    self.last_max_accel = accel_alpha * self.last_max_accel + (1 - accel_alpha) * target_max

    # Adaptive decel smoothing
    decel_alpha = self._adaptive_alpha(self.last_min_accel, target_min, DECEL_ALPHA_BASE, DECEL_ALPHA_MIN, DECEL_ALPHA_SCALE)
    smoothed_decel = decel_alpha * self.last_min_accel + (1 - decel_alpha) * target_min

    raw_change = smoothed_decel - self.last_min_accel
    limit = (MAX_DECEL_INCREASE_RATE if raw_change < 0 else MAX_DECEL_DECREASE_RATE) * DT_MDL
    self.last_min_accel += np.clip(raw_change, -limit, limit)

    gap = max(0.1, abs(self.last_max_accel) * 0.05)
    self.last_min_accel = min(self.last_min_accel, self.last_max_accel - gap)

    return float(self.last_min_accel), float(self.last_max_accel)

  def get_min_accel(self, v_ego: float) -> float:
    return self.get_accel_limits(v_ego)[0]

  def get_max_accel(self, v_ego: float) -> float:
    return self.get_accel_limits(v_ego)[1]

  def is_enabled(self) -> bool:
    return self._enabled

  def set_enabled(self, enabled: bool):
    self._enabled = enabled
    self.params.put_bool('AccelPersonalityEnabled', enabled)

  def toggle_enabled(self) -> bool:
    current = self._enabled
    self.set_enabled(not current)
    return not current

  def reset(self):
    self._accel_personality = AccelPersonality.normal
    self.params.put('AccelPersonality', AccelPersonality.normal)
    self.frame = 0
    self.last_max_accel = 2.0
    self.last_min_accel = -0.01
    self.first_run = True
