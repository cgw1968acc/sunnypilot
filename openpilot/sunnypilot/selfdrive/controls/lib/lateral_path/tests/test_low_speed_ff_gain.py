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

from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.gains import get_low_speed_ff_gain

KPH = 1 / 3.6


def test_full_gain_between_30_and_60_kph():
  for v in (30, 40, 50, 60):
    assert abs(get_low_speed_ff_gain(v * KPH) - 1.12) < 1e-9


def test_stock_outside_the_band():
  for v in (0, 10, 25, 70, 90, 120):
    assert get_low_speed_ff_gain(v * KPH) == 1.0


def test_continuous_no_step():
  vs = np.linspace(0, 100, 1001) * KPH
  g = [get_low_speed_ff_gain(v) for v in vs]
  assert max(abs(a - b) for a, b in zip(g[1:], g[:-1], strict=True)) < 0.003
  assert min(g) >= 1.0 and max(g) <= 1.12 + 1e-9
