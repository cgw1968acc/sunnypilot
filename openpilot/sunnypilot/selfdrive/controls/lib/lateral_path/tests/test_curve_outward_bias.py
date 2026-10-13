import unittest

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.curve_bias import (apply_curve_outward_bias,
                                                                                 CURVE_OUTWARD_DEADZONE,
                                                                                 CURVE_OUTWARD_V_BP,
                                                                                 CURVE_OUTWARD_FRAC_V)

V30, V40, V70 = 30 * CV.KPH_TO_MS, 40 * CV.KPH_TO_MS, 70 * CV.KPH_TO_MS


class TestCurveOutwardBias(unittest.TestCase):
  def test_straights_and_small_corrections_untouched(self):
    for v in (V30, V40, V70, 120 * CV.KPH_TO_MS):
      for k in (0.0, 0.0003, CURVE_OUTWARD_DEADZONE, -CURVE_OUTWARD_DEADZONE, -0.0004):
        self.assertEqual(apply_curve_outward_bias(k, v), k)

  def test_high_speed_curves_relaxed_outward_and_sign_preserved(self):
    for k in (0.0022, 0.0067, 0.012, -0.0067, -0.012):
      out = apply_curve_outward_bias(k, V70)
      self.assertLess(abs(out), abs(k))            # commands a wider radius
      self.assertGreater(out * k, 0.0)              # same direction

  def test_hand_over_between_30_and_40_kmh(self):
    # 2026-10-13: -0.01 (inward) at 30 km/h, 0 at 35 km/h, the outward bias 0.03 at 40 km/h
    for k in (0.006, 0.02, -0.02):
      self.assertAlmostEqual(apply_curve_outward_bias(k, V40), k * (1.0 - 0.03), places=9)
      self.assertEqual(apply_curve_outward_bias(k, 35 * CV.KPH_TO_MS), k)  # 35 km/h point = 0 (owner 2026-10-13)

  def test_low_speed_curves_slightly_inward(self):
    # 30 km/h and below: 1% more than the model curvature (owner 2026-10-13; was exactly the model since 2026-10-06)
    for v in (V30, 20 * CV.KPH_TO_MS):
      for k in (0.006, 0.02, -0.02):
        self.assertAlmostEqual(apply_curve_outward_bias(k, v), k * 1.01, places=12)

  def test_schedule_is_monotonic_and_bounded(self):
    self.assertEqual(list(CURVE_OUTWARD_V_BP), sorted(CURVE_OUTWARD_V_BP))
    self.assertEqual(list(CURVE_OUTWARD_FRAC_V), sorted(CURVE_OUTWARD_FRAC_V))
    self.assertEqual([round(f, 3) for f in CURVE_OUTWARD_FRAC_V], [-0.01, 0.0, 0.03, 0.063, 0.09, 0.09, 0.155, 0.155, 0.155])
    self.assertGreaterEqual(min(CURVE_OUTWARD_FRAC_V), -0.1)
    self.assertLessEqual(max(CURVE_OUTWARD_FRAC_V), 0.25)
    for k in (0.05, -0.05):
      self.assertLessEqual(abs(k - apply_curve_outward_bias(k, V70)), max(CURVE_OUTWARD_FRAC_V) * abs(k) + 1e-9)


if __name__ == "__main__":
  unittest.main()
