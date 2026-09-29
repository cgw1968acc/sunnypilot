import unittest
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import (get_cruise_accel, A_CRUISE_MAX_BP, J_CRUISE_VALS,
                                                                    J_CRUISE_EASE_DOWN, V_CRUISE_EASE_MIN)
import numpy as np

CP = SimpleNamespace(steerRatio=15.0, wheelbase=2.7)


def run(v_ego, v_cruise, a0, seconds, e2e=False):
  a = a0
  out = [a]
  for _ in range(int(seconds / DT_MDL)):
    a = get_cruise_accel(e2e, v_cruise, v_ego, a, 0.0, CP, DT_MDL, -0.35, True)
    out.append(a)
  return out


class TestCruiseSetSpeedEase(unittest.TestCase):
  def test_highway_set_speed_drop_eases_off_slowly(self):
    v = 105 * CV.KPH_TO_MS
    a = run(v, v - 0.35, 0.12, 3.0)          # accel-controller style target: coast 0.35 below v_ego
    rate = (a[0] - a[int(1.0 / DT_MDL)]) / 1.0
    self.assertAlmostEqual(rate, J_CRUISE_EASE_DOWN, delta=0.02)
    self.assertGreater(a[int(0.5 / DT_MDL)], 0.0)       # still on the throttle half a second in
    self.assertAlmostEqual(a[-1], -0.35, delta=0.02)    # and at the coast target by 3 s

  def test_below_90_kmh_keeps_stock_jerk(self):
    v = 70 * CV.KPH_TO_MS
    a = run(v, v - 0.35, 0.12, 1.0)
    j = float(np.interp(v, A_CRUISE_MAX_BP, J_CRUISE_VALS))
    self.assertAlmostEqual(a[0] - a[int(0.5 / DT_MDL)], min(0.47, j * 0.5), delta=0.03)

  def test_release_and_e2e_unaffected(self):
    v = 105 * CV.KPH_TO_MS
    up = run(v, v + 2.0, -0.35, 1.0)           # target above v_ego: normal upward jerk
    j = float(np.interp(v, A_CRUISE_MAX_BP, J_CRUISE_VALS))
    self.assertGreater(up[int(0.5 / DT_MDL)] - up[0], 0.9 * j * 0.5)
    e2e = run(v, v - 0.35, 0.12, 0.5, e2e=True)
    self.assertLess(e2e[-1], 0.12 - 0.9 * J_CRUISE_EASE_DOWN * 0.5)

  def test_threshold(self):
    self.assertAlmostEqual(V_CRUISE_EASE_MIN, 25.0, places=2)


if __name__ == "__main__":
  unittest.main()
