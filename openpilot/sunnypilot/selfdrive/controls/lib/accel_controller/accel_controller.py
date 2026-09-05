"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import get_sanitize_int_param

AccelProfile = custom.LongitudinalPlanSP.AccelController.Profile

MAX_ACCEL_BREAKPOINTS = [0., 3., 12,  24., 36.]  # m/s
MAX_ACCEL_PROFILES = {
  AccelProfile.eco:    [1.75, 1.38, 0.30, 0.11, 0.10],
  AccelProfile.normal: [1.85, 1.60, 0.50, 0.20, 0.15],
  AccelProfile.sport:  [2.00, 2.00, 1.20, 0.50, 0.30],
}
COMFORT_JERK = {AccelProfile.eco: 0.15, AccelProfile.normal: 0.25, AccelProfile.sport: 0.38}  # m/s^3
TARGET_SPEED_DEADBAND = 0.2  # m/s
LAUNCH_FLOOR_BREAKPOINTS = [1.0, 3.0]  # m/s
LAUNCH_FLOOR_VALUES = [1.2, 0.0]  # m/s^2


class AccelController:
  def __init__(self):
    self.params = Params()
    self.frame = 0
    self._profile = get_sanitize_int_param("AccelPersonality", AccelProfile.eco, AccelProfile.sport, self.params)
    self._enabled = self.params.get_bool("AccelPersonalityEnabled")

  def update(self) -> None:
    self.frame += 1
    if self.frame % int(1.0 / DT_MDL) == 0:
      self._profile = get_sanitize_int_param("AccelPersonality", AccelProfile.eco, AccelProfile.sport, self.params)
      self._enabled = self.params.get_bool("AccelPersonalityEnabled")

  @property
  def profile(self) -> int:
    return self._profile

  def is_enabled(self) -> bool:
    return self._enabled

  def get_max_accel(self, v_ego: float) -> float:
    return float(np.interp(max(0.0, v_ego), MAX_ACCEL_BREAKPOINTS, MAX_ACCEL_PROFILES[self._profile]))

  def get_comfort_accel(self, v_ego: float, v_target: float) -> float:
    v_ego = max(0.0, v_ego)
    speed_error = v_target - v_ego
    error = max(0.0, abs(speed_error) - TARGET_SPEED_DEADBAND)
    if not error > 0.0:
      return 0.0

    accel = min(error, float(np.sqrt(COMFORT_JERK[self._profile] * error)))
    if speed_error < 0.0:
      return -accel

    launch = float(np.interp(v_ego, LAUNCH_FLOOR_BREAKPOINTS, LAUNCH_FLOOR_VALUES))
    return min(max(accel, launch), error, self.get_max_accel(v_ego))

  def get_cruise_target(self, v_ego: float, v_target: float) -> float:
    if not np.isfinite(v_target) or v_target <= 0.0:
      return v_target

    return float(v_ego + self.get_comfort_accel(v_ego, v_target))
