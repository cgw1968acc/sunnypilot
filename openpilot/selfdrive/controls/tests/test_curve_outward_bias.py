import math
import unittest

from openpilot.selfdrive.controls.lib.latcontrol_torque import (apply_curve_outward_bias, CURVE_OUTWARD_FRAC,
                                                                CURVE_OUTWARD_DEADZONE)


class TestCurveOutwardBias(unittest.TestCase):
  def test_straights_and_small_corrections_untouched(self):
    for k in (0.0, 0.001, CURVE_OUTWARD_DEADZONE, -CURVE_OUTWARD_DEADZONE, 0.0025):
      self.assertEqual(apply_curve_outward_bias(k), k)

  def test_curves_relaxed_outward_and_sign_preserved(self):
    for k in (0.0067, 0.012, -0.0067, -0.012):
      out = apply_curve_outward_bias(k)
      self.assertLess(abs(out), abs(k))            # commands a wider radius
      self.assertEqual(math.copysign(1, out), math.copysign(1, k))  # same turn direction

  def test_reduction_is_proportional_to_curvature_over_the_deadzone(self):
    for k in (0.006, 0.010, 0.02):
      over = k - CURVE_OUTWARD_DEADZONE
      self.assertAlmostEqual(apply_curve_outward_bias(k), k - CURVE_OUTWARD_FRAC * over, places=9)

  def test_bias_is_bounded(self):
    # never flips sign or over-relaxes: reduction is at most CURVE_OUTWARD_FRAC of the curvature
    for k in (0.05, -0.05):
      self.assertLessEqual(abs(k - apply_curve_outward_bias(k)), CURVE_OUTWARD_FRAC * abs(k) + 1e-9)


if __name__ == "__main__":
  unittest.main()
