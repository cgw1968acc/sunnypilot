#!/usr/bin/env python3
# The MIT License
#
# Copyright (c) 2019-, Rick Lan, dragonpilot community, and a number of other of contributors.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# Last updated: July 1, 2024

from cereal import custom
from openpilot.common.numpy_fast import interp

AccelPersonality = custom.AccelerationPersonality

# accel personality by @arne182 modified by cgw

# accel personality by @arne182 modified by cgw
_DP_CRUISE_MIN_V =       [-0.06, -0.06,  -0.220,  -0.220,  -0.34,  -0.34,  -0.64,  -0.64,  -0.82]
_DP_CRUISE_MIN_V_ECO =   [-0.06, -0.06,  -0.215,  -0.215,  -0.32,  -0.32,  -0.62,  -0.62,  -0.80]
_DP_CRUISE_MIN_V_SPORT = [-0.06, -0.06,  -0.225,  -0.225,  -0.36,  -0.36,  -0.66,  -0.66,  -0.84]
_DP_CRUISE_MIN_BP =      [0.,     5.55,  5.56,    18.,     18.01,  27.77,  27.78,  33.,    33.01]

_DP_CRUISE_MAX_V =       [3.5, 3.5, 3.4, 2.4,  1.4,  .91,  .70,  .51,  .42,  .13]
_DP_CRUISE_MAX_V_ECO =   [3.5, 3.5, 3.3, 2.2,  1.1,  .75,  .53,  .42,  .31,  .085]
_DP_CRUISE_MAX_V_SPORT = [3.5, 3.5, 3.5, 2.8,  1.6,  1.1,  .96,  .78,  .60,  .4]
_DP_CRUISE_MAX_BP =      [0.,  1.,  6.,  8.,   11.,  15.,  20.,  25.,  30.,  55.]



class AccelController:
  def __init__(self):
    self._personality = AccelPersonality.stock

  def _dp_calc_cruise_accel_limits(self, v_ego: float) -> tuple[float, float]:
    if self._personality == AccelPersonality.eco:
      min_v = _DP_CRUISE_MIN_V_ECO
      max_v = _DP_CRUISE_MAX_V_ECO
    elif self._personality == AccelPersonality.sport:
      min_v = _DP_CRUISE_MIN_V_SPORT
      max_v = _DP_CRUISE_MAX_V_SPORT
    else:
      min_v = _DP_CRUISE_MIN_V
      max_v = _DP_CRUISE_MAX_V

    a_cruise_min = interp(v_ego, _DP_CRUISE_MIN_BP, min_v)
    a_cruise_max = interp(v_ego, _DP_CRUISE_MAX_BP, max_v)

    return a_cruise_min, a_cruise_max

  def get_accel_limits(self, v_ego: float, accel_limits: list[float]) -> tuple[float, float]:
    return accel_limits if self._personality == AccelPersonality.stock else self._dp_calc_cruise_accel_limits(v_ego)

  def is_enabled(self, accel_personality: int = AccelPersonality.stock) -> bool:
    self._personality = accel_personality
    return self._personality != AccelPersonality.stock
