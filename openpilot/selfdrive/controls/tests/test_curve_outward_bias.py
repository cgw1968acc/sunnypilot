import unittest

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.latcontrol_torque import (apply_curve_outward_bias, CURVE_OUTWARD_DEADZONE,
                                                                CURVE_OUTWARD_V_BP, CURVE_OUTWARD_FRAC_V)

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

  def test_zero_at_40_kmh(self):
    # the hand-over point between the inward (low speed) and outward (high speed) schedules: exactly the model curvature
    for k in (0.006, 0.02, -0.02):
      self.assertEqual(apply_curve_outward_bias(k, V40), k)

  def test_low_speed_curves_biased_inward(self):
    # 30 km/h and below: a little MORE curvature than the model (Altis runs wide in tight low-speed curves)
    for v in (V30, 20 * CV.KPH_TO_MS):
      for k in (0.006, 0.02, -0.02):
        out = apply_curve_outward_bias(k, v)
        self.assertGreater(abs(out), abs(k))
        self.assertGreater(out * k, 0.0)
        self.assertAlmostEqual(out, k * (1.0 - CURVE_OUTWARD_FRAC_V[0]), places=9)

  def test_schedule_is_monotonic_and_bounded(self):
    self.assertEqual(list(CURVE_OUTWARD_V_BP), sorted(CURVE_OUTWARD_V_BP))
    self.assertEqual(list(CURVE_OUTWARD_FRAC_V), sorted(CURVE_OUTWARD_FRAC_V))
    self.assertGreaterEqual(min(CURVE_OUTWARD_FRAC_V), -0.1)
    self.assertLessEqual(max(CURVE_OUTWARD_FRAC_V), 0.25)
    for k in (0.05, -0.05):
      self.assertLessEqual(abs(k - apply_curve_outward_bias(k, V70)), max(CURVE_OUTWARD_FRAC_V) * abs(k) + 1e-9)


if __name__ == "__main__":
  unittest.main()
