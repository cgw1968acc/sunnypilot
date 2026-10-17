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


def test_downhill_plus_10_capped_at_120_and_smooth():
  sc = flat_learnt()
  assert 100 < run(sc, 20 * 10, 100, 0.5 - 2.0) < 105   # after 10 s: only ~3.3 km/h (0.33 km/h/s)
  assert abs(run(sc, 20 * 30, 100, 0.5 - 2.0) - 110) < 0.5
  assert abs(run(flat_learnt(), 20 * 60, 115, 0.5 - 2.0) - 120) < 1e-6
  assert abs(run(flat_learnt(), 20 * 60, 125, 0.5 - 2.0) - 125) < 1e-6  # set above 120: no increase


def test_uphill_minus_10_floored_at_70():
  assert abs(run(flat_learnt(), 20 * 60, 100, 0.5 + 2.0) - 90) < 0.5
  assert abs(run(flat_learnt(), 20 * 60, 75, 0.5 + 2.0) - 70) < 1e-6
  assert abs(run(flat_learnt(), 20 * 60, 65, 0.5 + 2.0, v_kph=65) - 65) < 1e-6  # set below 70: kept


def test_inactive_or_slow_or_disabled(monkeypatch):
  assert abs(run(flat_learnt(), 600, 100, -2.0, active=False) - 100) < 1e-6
  assert abs(run(flat_learnt(), 600, 100, -2.0, v_kph=50) - 100) < 1e-6
  monkeypatch.setattr(sc_mod, "ENABLED", False)
  assert abs(run(flat_learnt(), 600, 100, -2.0) - 100) < 1e-6


def test_returns_smoothly_after_the_slope():
  sc = flat_learnt()
  run(sc, 20 * 40, 100, 0.5 - 2.0)
  assert 105 < run(sc, 20 * 10, 100, 0.5) < 108  # 10 s after the descent ends still easing back
