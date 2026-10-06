import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.curvature_shaping import HighwayCurvatureSmoother

DT = 0.01


def swing_amplitude(v_ego, period, n=3000):
  sm = HighwayCurvatureSmoother(DT)
  out = [sm.update(1e-3 * np.sin(2 * np.pi * i * DT / period), v_ego, True, False) for i in range(n)]
  return np.max(np.abs(out[n // 2:])) / 1e-3


def test_untouched_below_70kph():
  assert abs(swing_amplitude(60 / 3.6, 3.0) - 1.0) < 1e-6


def test_highway_swing_is_reduced_and_slow_motion_kept():
  fast = swing_amplitude(100 / 3.6, 2.0)
  slow = swing_amplitude(100 / 3.6, 30.0)
  assert 0.5 < fast < 0.6   # 2 s swing: ~-46%
  assert slow > 0.99        # a 30 s drift / a steady curve is kept


def test_steady_curve_reaches_full_curvature():
  sm = HighwayCurvatureSmoother(DT)
  for _ in range(500):
    out = sm.update(2e-3, 100 / 3.6, True, False)
  assert abs(out - 2e-3) < 1e-6


def test_bypassed_in_lane_change_and_inactive():
  sm = HighwayCurvatureSmoother(DT)
  sm.update(0.0, 100 / 3.6, True, False)
  assert sm.update(5e-3, 100 / 3.6, True, True) == 5e-3
  assert sm.update(-5e-3, 100 / 3.6, False, False) == -5e-3
