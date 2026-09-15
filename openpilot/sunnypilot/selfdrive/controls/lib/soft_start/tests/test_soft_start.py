import unittest

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.soft_start.soft_start import (CruiseSoftStart, soft_start_up_jerk, apply_cruise_jerk_limits,
                                                                              ACCEL_IDLE_THRESHOLD)

DT = 0.05
J_CRUISE = 1.06  # planner J_CRUISE at ~15 m/s


def ramp(a0, target, secs=3.0, j_cruise=J_CRUISE):
  ss = CruiseSoftStart(DT)
  a = a0
  out = []
  for _ in range(int(secs / DT)):
    t = ss.update(a)
    a = apply_cruise_jerk_limits(target, a, j_cruise, soft_start_up_jerk(t, j_cruise), DT)
    out.append(a)
  return out


class TestCruiseSoftStart(unittest.TestCase):
  def test_first_second_is_gentle_then_blends_to_target(self):
    out = ramp(0.0, 0.45)
    self.assertLess(out[int(0.5 / DT) - 1], 0.16)
    self.assertLess(out[int(1.0 / DT) - 1], 0.30)
    self.assertAlmostEqual(out[int(1.5 / DT) - 1], 0.45, places=2)

  def test_large_step_reaches_full_accel_within_two_seconds(self):
    out = ramp(0.0, 1.10, j_cruise=1.32)
    self.assertLess(out[int(1.0 / DT) - 1], 0.30)
    self.assertAlmostEqual(out[int(2.0 / DT) - 1], 1.10, places=2)

  def test_jerk_never_exceeds_the_stock_limit(self):
    out = ramp(-0.5, 1.5, j_cruise=1.2)
    jerks = np.diff([-0.5] + out) / DT
    self.assertLessEqual(max(jerks), 1.2 + 1e-6)

  def test_recovery_from_deceleration_up_to_idle_is_not_slowed(self):
    out = ramp(-0.44, 0.47, j_cruise=1.10)
    # -0.44 -> idle (0.05) at the stock jerk takes 0.45 s; the soft start only applies above idle
    t_idle = next(i * DT for i, a in enumerate(out) if a >= ACCEL_IDLE_THRESHOLD)
    self.assertLess(t_idle, 0.5)

  def test_releasing_acceleration_stays_fast(self):
    ss = CruiseSoftStart(DT)
    a = 0.45
    for _ in range(10):
      a = apply_cruise_jerk_limits(-0.5, a, 1.1, soft_start_up_jerk(ss.update(a), 1.1), DT)
    self.assertAlmostEqual(a, 0.45 - 1.1 * 0.5, places=2)

  def test_reset_and_unarmed_state_use_stock_jerk(self):
    ss = CruiseSoftStart(DT)
    self.assertIsNone(ss.t_accelerating)
    self.assertEqual(soft_start_up_jerk(None, 1.06), 1.06)
    ss.update(0.3)
    self.assertIsNone(ss.t_accelerating)  # never left idle since reset: stays unarmed until a_prev drops below idle
    ss.update(0.0)
    self.assertEqual(ss.t_accelerating, 0.0)
    ss.update(0.2)
    self.assertAlmostEqual(ss.t_accelerating, DT)
    ss.reset()
    self.assertIsNone(ss.t_accelerating)


if __name__ == "__main__":
  unittest.main()
