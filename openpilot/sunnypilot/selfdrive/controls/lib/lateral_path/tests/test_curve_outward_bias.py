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
    # tncr19 Cross table (ported 2026-10-15): -0.05 (inward) at 30 km/h, 0.02 outward at 40, zero at ~37 km/h
    for k in (0.006, 0.02, -0.02):
      self.assertAlmostEqual(apply_curve_outward_bias(k, V40), k * (1.0 - 0.02), places=9)
      v_zero = V30 + (V40 - V30) * (0.05 / 0.07)
      self.assertAlmostEqual(apply_curve_outward_bias(k, v_zero), k, places=9)

  def test_low_speed_curves_biased_inward(self):
    # 30 km/h and below: 5% more than the model curvature (tncr18/tncr19 Cross tune)
    for v in (V30, 20 * CV.KPH_TO_MS):
      for k in (0.006, 0.02, -0.02):
        out = apply_curve_outward_bias(k, v)
        self.assertGreater(abs(out), abs(k))
        self.assertGreater(out * k, 0.0)
        self.assertAlmostEqual(out, k * (1.0 - CURVE_OUTWARD_FRAC_V[0]), places=9)

  def test_schedule_is_monotonic_and_bounded(self):
    self.assertEqual(list(CURVE_OUTWARD_V_BP), sorted(CURVE_OUTWARD_V_BP))
    # monotonic except 60 -> 70 km/h: the owner raised 50-60 by 0.01 on 2026-10-22 (60: 0.095, 70 stays 0.09)
    f = list(CURVE_OUTWARD_FRAC_V)
    self.assertTrue(all(b >= a for i, (a, b) in enumerate(zip(f, f[1:], strict=False)) if i != 3))
    self.assertEqual([round(f, 3) for f in CURVE_OUTWARD_FRAC_V], [-0.05, 0.02, 0.07, 0.095, 0.09, 0.155, 0.155, 0.155])  # 10-22: 50-60 +0.01
    self.assertGreaterEqual(min(CURVE_OUTWARD_FRAC_V), -0.1)
    self.assertLessEqual(max(CURVE_OUTWARD_FRAC_V), 0.25)
    for k in (0.05, -0.05):
      self.assertLessEqual(abs(k - apply_curve_outward_bias(k, V70)), max(CURVE_OUTWARD_FRAC_V) * abs(k) + 1e-9)


if __name__ == "__main__":
  unittest.main()
