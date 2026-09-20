import unittest

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap import (StopGapGovernor, STOP_GAP, A_NEG_MAX, V_ENGAGE, S_ENGAGE,
                                                                          A_HOLD, A_MIN, RELEASE_RATE)

DT = 0.05


def mpc_like(v, gap):
  """Stand-in for the lead MPC on the last metres: roughly the decel needed, never lighter than 0.3."""
  return -float(np.clip(v * v / (2.0 * max(gap - 3.0, 0.5)), 0.3, 1.2))


def approach(v0, gap0, secs=15.0, lag=0.4, gain=1.0, creep=0.0, mpc=mpc_like):
  """Closed loop: governor (or the MPC stand-in when it declines) drives a first-order lagged plant that brakes
  `gain` times the request and creeps at +creep when the request is lighter than -0.25. The car's stopping state
  (should_stop: v < 0.25 and target < 0.1) holds the last target; the loop then runs on for 3 s to look for creep."""
  gov = StopGapGovernor(DT)
  v, gap, a = v0, gap0, mpc(v0, gap0)
  targets, used = [], []
  t, t_stop, hold, crept, gap_stop = 0.0, None, None, False, None
  while t < secs:
    target = gov.update(True, v, True, gap, 0.0, mpc(v, gap))
    used.append(target is not None)
    if target is None:
      target = mpc(v, gap)
    if t_stop is not None:
      target = hold
    elif v < 0.25 and target < 0.1:
      t_stop, hold, gap_stop = t, target, gap
    targets.append(target)
    plant = target * gain if (target < -0.25 and v < 1.5) else (creep if target > -0.25 else target)
    a += (plant - a) * DT / lag
    v = max(v + a * DT, 0.0)
    gap -= v * DT
    t += DT
    if t_stop is not None and t > t_stop + 0.5 and v > 0.05:
      crept = True
    if t_stop is not None and t > t_stop + 3.0:
      break
  return {"gap": gap_stop if gap_stop is not None else gap, "hold": hold, "crept": crept, "targets": np.array(targets), "used": used}


class TestStopGapGovernor(unittest.TestCase):
  def test_stops_at_the_gap_from_a_normal_approach(self):
    r = approach(2.5, 8.5)
    self.assertAlmostEqual(r['gap'], STOP_GAP + 0.15, delta=0.2)
    self.assertTrue(any(r['used']))
    self.assertLess(r['targets'].max(), -A_MIN + 1e-9)  # never released on the approach
    self.assertGreater(r['targets'].min(), -1.2)  # no hard catch-up braking

  def test_still_lands_at_the_gap_when_the_pcm_overbrakes_and_the_car_creeps(self):
    for v0, gap0 in ((2.5, 8.5), (3.0, 7.5), (2.0, 6.5), (1.2, 5.0)):
      r = approach(v0, gap0, gain=1.6, creep=0.35)
      self.assertAlmostEqual(r['gap'], STOP_GAP + 0.3, delta=0.3, msg=f'{v0=} {gap0=}')
      self.assertFalse(r['crept'], msg=f'{v0=} {gap0=}')
      self.assertLessEqual(r['hold'], -A_HOLD + 1e-9, msg=f'{v0=} {gap0=}')

  def test_slow_arrival_is_a_gentle_constant_brake_not_a_release(self):
    r = approach(0.6, 4.3, gain=1.6, creep=0.35)
    self.assertLess(r['targets'].max(), -A_MIN + 1e-9)
    self.assertFalse(r['crept'])
    self.assertLess(r['gap'], 4.3)

  def test_release_from_the_mpc_is_rate_limited(self):
    gov = StopGapGovernor(DT)
    a0 = gov.update(True, 1.0, True, STOP_GAP + 5.0, 0.0, -0.9)  # needs only 0.1 m/s^2 but was braking at 0.9
    self.assertAlmostEqual(a0, -0.9 + RELEASE_RATE * DT, places=6)
    a1 = gov.update(True, 1.0, True, STOP_GAP + 5.0, 0.0, -0.9)
    self.assertAlmostEqual(a1, a0 + RELEASE_RATE * DT, places=6)

  def test_holds_a_firm_brake_below_v_hold_and_past_the_point(self):
    gov = StopGapGovernor(DT)
    for v, s in ((0.36, 0.3), (0.15, 0.1), (0.0, 0.05), (0.0, -0.1), (0.2, -0.2), (0.49, 2.0)):
      gov.reset()
      self.assertLessEqual(gov.update(True, v, True, STOP_GAP + s, 0.0, -1.0), -A_HOLD + 1e-9)

  def test_brakes_harder_when_fast_and_close(self):
    gov = StopGapGovernor(DT)
    self.assertLess(gov.update(True, 2.0, True, STOP_GAP + 1.0, 0.0, -1.0), -1.5)
    self.assertGreaterEqual(gov.update(True, 3.0, True, STOP_GAP - 1.0, 0.0, -2.0), A_NEG_MAX)

  def test_moving_lead_hands_back_to_the_mpc(self):
    gov = StopGapGovernor(DT)
    self.assertIsNotNone(gov.update(True, 1.0, True, 5.0, 0.0, -0.5))
    self.assertIsNone(gov.update(True, 1.0, True, 5.0, 0.5, -0.5))
    self.assertFalse(gov.engaged)
    self.assertIsNone(gov.update(True, 1.0, False, 5.0, 0.0, -0.5))
    self.assertIsNone(gov.update(False, 1.0, True, 5.0, 0.0, -0.5))

  def test_engage_window_with_hysteresis(self):
    gov = StopGapGovernor(DT)
    self.assertIsNone(gov.update(True, V_ENGAGE + 0.5, True, STOP_GAP + 5.0, 0.0, -0.5))
    self.assertIsNone(gov.update(True, 2.0, True, STOP_GAP + S_ENGAGE + 0.5, 0.0, -0.5))
    self.assertIsNotNone(gov.update(True, 2.0, True, STOP_GAP + 5.0, 0.0, -0.5))
    self.assertIsNotNone(gov.update(True, V_ENGAGE + 0.5, True, STOP_GAP + 5.0, 0.0, -0.5))
    self.assertIsNotNone(gov.update(True, 2.0, True, STOP_GAP + S_ENGAGE + 0.5, 0.0, -0.5))
    self.assertIsNone(gov.update(True, V_ENGAGE + 1.5, True, STOP_GAP + 5.0, 0.0, -0.5))


if __name__ == "__main__":
  unittest.main()
