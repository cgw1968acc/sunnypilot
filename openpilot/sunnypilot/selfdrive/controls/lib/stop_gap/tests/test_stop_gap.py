import unittest

from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap import (StopGapGovernor, STOP_GAP, A_POS_MAX, A_NEG_MAX,
                                                                          V_ENGAGE, S_ENGAGE)

DT = 0.05


def approach(v0, gap0, secs=15.0, lag=0.3, mpc=lambda v, gap: -0.9):
  """Closed loop: governor (or the MPC stand-in when it declines) drives a first-order lagged plant. The car's
  stopping state (should_stop: v < 0.25 and target < 0.1) is modelled as a stop within 0.2 s."""
  gov = StopGapGovernor()
  v, gap, a = v0, gap0, 0.0
  used, targets = [], []
  for _ in range(int(secs / DT)):
    target = gov.update(True, v, True, gap, 0.0)
    used.append(target is not None)
    if target is None:
      target = mpc(v, gap)
    targets.append(target)
    if v < 0.25 and target < 0.1:
      gap -= v * 0.1  # roll of the stopping state
      v = 0.0
      break
    a += (target - a) * DT / lag
    v = max(v + a * DT, 0.0)
    gap -= v * DT
  return v, gap, used, targets


class TestStopGapGovernor(unittest.TestCase):
  def test_stops_at_the_gap_from_a_normal_approach(self):
    v, gap, used, targets = approach(2.5, 8.5)
    self.assertEqual(v, 0.0)
    self.assertAlmostEqual(gap, STOP_GAP + 0.1, delta=0.15)
    self.assertTrue(any(used))
    self.assertLessEqual(max(targets), 0.0)  # never accelerates on the approach
    self.assertGreater(min(targets), -1.2)  # and no sharp catch-up braking

  def test_stops_at_the_gap_when_the_mpc_would_have_stopped_long(self):
    # ego already slow with 1.3 m to spare: rolls on to the stop point instead of stopping at 4.8 m
    v, gap, _, _ = approach(0.6, 4.3, mpc=lambda v, gap: -1.5)
    self.assertEqual(v, 0.0)
    self.assertAlmostEqual(gap, STOP_GAP + 0.1, delta=0.15)

  def test_stops_at_the_gap_when_arriving_fast(self):
    v, gap, _, targets = approach(3.0, 7.5)
    self.assertEqual(v, 0.0)
    self.assertAlmostEqual(gap, STOP_GAP + 0.1, delta=0.2)
    self.assertGreater(min(targets), -1.6)

  def test_deceleration_tapers_out_at_the_end(self):
    gov = StopGapGovernor()
    # crawling in with 0.3 m to go at the profile speed: barely any braking asked for
    a = gov.update(True, 0.36, True, STOP_GAP + 0.3, 0.0)
    self.assertGreater(a, -0.5)
    self.assertLess(a, 0.0)

  def test_never_positive_when_stopped(self):
    gov = StopGapGovernor()
    self.assertLessEqual(gov.update(True, 0.0, True, STOP_GAP + 1.0, 0.0), 0.0)
    self.assertLessEqual(gov.update(True, 0.05, True, STOP_GAP + 1.0, 0.0), 0.0)
    self.assertLessEqual(gov.update(True, 0.5, True, STOP_GAP + 2.0, 0.0), 0.0)  # positive only close to the stop point
    self.assertLessEqual(gov.update(True, 0.3, True, STOP_GAP + 1.0, 0.0), A_POS_MAX)
    self.assertGreater(gov.update(True, 0.3, True, STOP_GAP + 1.0, 0.0), 0.0)

  def test_brakes_when_past_the_stop_point(self):
    gov = StopGapGovernor()
    self.assertLess(gov.update(True, 0.5, True, STOP_GAP - 0.5, 0.0), -0.5)
    self.assertGreaterEqual(gov.update(True, 3.0, True, STOP_GAP - 1.0, 0.0), A_NEG_MAX)

  def test_moving_lead_hands_back_to_the_mpc(self):
    gov = StopGapGovernor()
    self.assertIsNotNone(gov.update(True, 1.0, True, 5.0, 0.0))
    self.assertIsNone(gov.update(True, 1.0, True, 5.0, 0.5))
    self.assertFalse(gov.engaged)
    self.assertIsNone(gov.update(True, 1.0, False, 5.0, 0.0))
    self.assertIsNone(gov.update(False, 1.0, True, 5.0, 0.0))

  def test_engage_window_with_hysteresis(self):
    gov = StopGapGovernor()
    self.assertIsNone(gov.update(True, V_ENGAGE + 0.5, True, STOP_GAP + 5.0, 0.0))
    self.assertIsNone(gov.update(True, 2.0, True, STOP_GAP + S_ENGAGE + 0.5, 0.0))
    self.assertIsNotNone(gov.update(True, 2.0, True, STOP_GAP + 5.0, 0.0))
    # once engaged, a little more speed or distance does not drop it
    self.assertIsNotNone(gov.update(True, V_ENGAGE + 0.5, True, STOP_GAP + 5.0, 0.0))
    self.assertIsNotNone(gov.update(True, 2.0, True, STOP_GAP + S_ENGAGE + 0.5, 0.0))
    self.assertIsNone(gov.update(True, V_ENGAGE + 1.5, True, STOP_GAP + 5.0, 0.0))


if __name__ == "__main__":
  unittest.main()
