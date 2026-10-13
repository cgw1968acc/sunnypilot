"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.highway_friction import HighwayFrictionScale, get_highway_friction_scale


def test_schedule():
  assert get_highway_friction_scale(50 / 3.6) == 1.0
  assert abs(get_highway_friction_scale(65 / 3.6) - 1.2) < 1e-9
  assert get_highway_friction_scale(70 / 3.6) == 1.4
  assert abs(get_highway_friction_scale(85 / 3.6) - 1.45) < 1e-9
  assert get_highway_friction_scale(100 / 3.6) == 1.5
  assert get_highway_friction_scale(130 / 3.6) == 1.5


def test_no_compounding_and_follows_new_base():
  tp = SimpleNamespace(friction=0.1)
  h = HighwayFrictionScale()
  for _ in range(100):
    h.apply(tp, 100 / 3.6)
  assert abs(tp.friction - 0.15) < 1e-9
  h.apply(tp, 40 / 3.6)
  assert abs(tp.friction - 0.1) < 1e-9
  tp.friction = 0.2  # torqued / the override set a new value
  h.apply(tp, 100 / 3.6)
  assert abs(tp.friction - 0.3) < 1e-9


def test_float32_readback():
  class F32:
    def __init__(self, v): self._f = np.float32(v)
    @property
    def friction(self): return float(self._f)
    @friction.setter
    def friction(self, v): self._f = np.float32(v)
  tp = F32(0.13)
  h = HighwayFrictionScale()
  for _ in range(50):
    h.apply(tp, 90 / 3.6)
  assert abs(tp.friction - 0.13 * get_highway_friction_scale(90 / 3.6)) < 1e-6
