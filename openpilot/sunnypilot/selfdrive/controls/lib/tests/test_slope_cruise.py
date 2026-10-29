# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
import math

import openpilot.sunnypilot.selfdrive.controls.lib.slope_cruise as sc_mod
from openpilot.sunnypilot.selfdrive.controls.lib.slope_cruise import SlopeCruise

K = 1 / 3.6


def run(sc, n, set_kph, pitch_deg, v_kph=None, active=True):
  out = set_kph * K
  for _ in range(n):
    out = sc.update(set_kph * K, (v_kph or set_kph) * K, math.radians(pitch_deg), active)
  return out / K


def flat_learnt():
  sc = SlopeCruise()
  sc.flat = 0.5
  return sc


def test_flat_road_unchanged():
  assert abs(run(flat_learnt(), 600, 100, 0.5) - 100) < 1e-6


def test_downhill_no_longer_raises_the_target():
  # owner 2026-10-27: the descent allowance is removed
  for set_kph in (63, 100, 115):
    assert abs(run(flat_learnt(), 20 * 60, set_kph, 0.5 - 2.0, v_kph=max(set_kph, 85)) - set_kph) < 1e-6


def test_uphill_minus_10_floored_at_70():
  assert abs(run(flat_learnt(), 20 * 60, 100, 0.5 + 2.0) - 90) < 0.5
  assert abs(run(flat_learnt(), 20 * 60, 75, 0.5 + 2.0, v_kph=85) - 70) < 1e-6
  assert abs(run(flat_learnt(), 20 * 60, 65, 0.5 + 2.0, v_kph=65) - 65) < 1e-6  # set below 70: kept


def test_inactive_or_slow_or_disabled(monkeypatch):
  assert abs(run(flat_learnt(), 600, 100, -2.0, active=False) - 100) < 1e-6
  assert abs(run(flat_learnt(), 600, 100, 3.0, v_kph=75) - 100) < 1e-6  # below 80 km/h (mountain roads): untouched
  monkeypatch.setattr(sc_mod, "ENABLED", False)
  assert abs(run(flat_learnt(), 600, 100, -2.0) - 100) < 1e-6


def test_returns_smoothly_after_the_slope():
  sc = flat_learnt()
  run(sc, 20 * 40, 100, 0.5 + 2.0)
  assert 92 < run(sc, 20 * 10, 100, 0.5) < 95  # 10 s after the climb ends still easing back
