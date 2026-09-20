import unittest

from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap import (StopGapGovernor, STOP_GAP, A_NEG_MAX, V_ENGAGE, S_ENGAGE,
                                                                          RELEASE_FRACTION, a_profile)

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

  def test_slow_arrival_is_not_pushed_on_and_never_released_to_zero(self):
    # ego already slow with 1.3 m to spare: the brake only gets lighter, it never lets go and never pushes
    v, gap, _, targets = approach(0.6, 4.3, mpc=lambda v, gap: -1.5)
    self.assertEqual(v, 0.0)
    self.assertLess(max(targets), 0.0)
    self.assertLess(gap, 4.3)

  def test_stops_at_the_gap_when_arriving_fast(self):
    v, gap, _, targets = approach(3.0, 7.5)
    self.assertEqual(v, 0.0)
    self.assertAlmostEqual(gap, STOP_GAP + 0.1, delta=0.2)
    self.assertGreater(min(targets), -1.6)

  def test_deceleration_tapers_out_at_the_end(self):
    gov = StopGapGovernor()
    # crawling in with 0.1 m to go near the profile speed: only a light brake left, and still a brake
    a = gov.update(True, 0.15, True, STOP_GAP + 0.1, 0.0)
    self.assertGreater(a, -0.4)
    self.assertLess(a, 0.0)

  def test_never_positive_and_never_lighter_than_half_the_profile(self):
    gov = StopGapGovernor()
    for v, gap in ((0.0, STOP_GAP + 1.0), (0.05, STOP_GAP + 1.0), (0.3, STOP_GAP + 1.0), (0.5, STOP_GAP + 2.0), (0.2, STOP_GAP + 5.0)):
      gov.reset()
      a = gov.update(True, v, True, gap, 0.0)
      self.assertLessEqual(a, RELEASE_FRACTION * a_profile(gap - STOP_GAP, gov.a_nom) + 1e-9)
      self.assertLessEqual(a, 0.0)

  def test_brake_only_eases_as_the_stop_nears(self):
    # a car braked harder than the profile (PCM overshoot) gets a lighter brake, monotonically, not a release
    _, _, _, targets = approach(1.2, 5.0, lag=0.3)
    self.assertLess(max(targets), 0.0)
    steps = [b - a for a, b in zip(targets, targets[1:], strict=False)]
    self.assertLess(max(steps), 0.1)  # no sudden release, per 50 ms frame

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
