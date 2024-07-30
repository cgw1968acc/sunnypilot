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
#_DP_CRUISE_MIN_V =       [-0.70,  -0.70,  -0.70,  -0.70,  -0.88,  -0.95,  -1.00]
#_DP_CRUISE_MIN_V_ECO =   [-0.68,  -0.68,  -0.68,  -0.68,  -0.86,  -0.90,  -1.00]
#_DP_CRUISE_MIN_V_SPORT = [-0.72,  -0.72,  -0.72,  -0.72,  -0.90,  -1.00,  -1.00]
#_DP_CRUISE_MIN_BP =      [0.,     5.32,   5.33,   15.99,  16.,    30.,    40.]

#_DP_CRUISE_MAX_V =       [3.2, 2.5, 2.5, 1.70, 1.05, .81,  .625, .42,  .348, .12]
#_DP_CRUISE_MAX_V_ECO =   [3.0, 2.0, 2.0, 1.4, .80,   .68,  .53,  .32,  .20,  .085]
#_DP_CRUISE_MAX_V_SPORT = [3.5, 3.5, 2.8, 2.4,  1.4,  1.0,  .89,  .75,  .50,  .2]
#_DP_CRUISE_MAX_BP =      [0.,  1.,  6.,  8.,   11.,  15.,  20.,  25.,  30.,  55.]

# accel personality by @arne182 modified by cgw
_DP_CRUISE_MIN_V =       [-0.065, -0.065,  -0.220,  -0.220,  -0.27,  -0.88,  -0.82]
_DP_CRUISE_MIN_V_ECO =   [-0.06, -0.06,  -0.215,  -0.215,  -0.26,  -0.80,  -0.80]
_DP_CRUISE_MIN_V_SPORT = [-0.07, -0.07,  -0.225,  -0.225,  -0.28,  -0.90,  -0.84]
_DP_CRUISE_MIN_BP =      [0.,     5.55,  5.56,    17.,     20.01,  30.01,  33.]

_DP_CRUISE_MAX_V =       [3.5, 3.5, 3.4, 2.6,  1.4,  .94,  .70,  .51,  .42,  .13]
_DP_CRUISE_MAX_V_ECO =   [3.5, 3.5, 3.3, 2.4,  1.2,  .78,  .53,  .42,  .31,  .085]
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
