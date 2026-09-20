import unittest

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap import (StopGapGovernor, STOP_GAP, A_NEG_MAX, V_ENGAGE, S_ENGAGE,
                                                                          A_HOLD, A_MIN, CREEP_LEAD_V, RELEASE_RATE)

DT = 0.05


def mpc_like(v_close, s):
  return -float(np.clip(v_close * v_close / (2.0 * max(s, 0.5)), 0.3, 1.2)) if v_close > 0 else -0.05


def run(v0, gap0, lead_v=None, secs=16.0, lag=0.4, gain=1.0, creep=0.0):
  if lead_v is None:
    lead_v = lambda t: 0.0  # noqa: E731
  """Closed loop against a first-order lagged plant that brakes `gain` times the request below 1.5 m/s and creeps at
  +creep when the request is lighter than -0.25. lead_v(t) is the lead's speed; the gap changes with both cars."""
  gov = StopGapGovernor(DT)
  v, gap = v0, gap0
  vl0 = lead_v(0.0)
  a = mpc_like(v0 - vl0, gap0 - STOP_GAP)
  targets, gaps, vs = [], [], []
  t, t_stop, hold, crept, gap_stop = 0.0, None, None, False, None
  while t < secs:
    vl = max(lead_v(t), 0.0)
    target = gov.update(True, v, True, gap, vl, mpc_like(v - vl, gap - STOP_GAP))
    if target is None:
      target = mpc_like(v - vl, gap - STOP_GAP)
    if t_stop is not None and vl < 0.05:
      target = hold
    elif v < 0.25 and vl < 0.05 and target < 0.1 and t_stop is None:
      t_stop, hold, gap_stop = t, target, gap
    targets.append(target)
    gaps.append(gap)
    vs.append(v)
    plant = target * gain if (target < -0.25 and v < 1.5) else (creep if target > -0.25 else target)
    a += (plant - a) * DT / lag
    v = max(v + a * DT, 0.0)
    gap += (vl - v) * DT
    t += DT
    if t_stop is not None and t > t_stop + 0.5 and v > 0.1 and vl < 0.05:
      crept = True
    if t_stop is not None and t > t_stop + 2.5 and vl < 0.05:
      break
  return {"gap": gap_stop if gap_stop is not None else gap, "hold": hold, "crept": crept,
          "targets": np.array(targets), "gaps": np.array(gaps), "vs": np.array(vs)}


class TestStopGapGovernor(unittest.TestCase):
  def test_stops_at_the_gap_from_a_normal_approach(self):
    r = run(2.5, 8.5)
    self.assertAlmostEqual(r['gap'], STOP_GAP + 0.15, delta=0.25)
    self.assertLess(r['targets'].max(), -A_MIN + 1e-9)
    self.assertGreater(r['targets'].min(), -1.2)

  def test_still_lands_at_the_gap_when_the_pcm_overbrakes_and_creeps(self):
    for v0, gap0 in ((2.5, 8.5), (3.0, 7.5), (2.0, 6.5), (1.2, 5.0)):
      r = run(v0, gap0, gain=1.6, creep=0.35)
      self.assertAlmostEqual(r['gap'], STOP_GAP + 0.3, delta=0.35, msg=f'{v0=} {gap0=}')
      self.assertFalse(r['crept'], msg=f'{v0=} {gap0=}')
      self.assertLessEqual(r['hold'], -A_HOLD + 1e-9, msg=f'{v0=} {gap0=}')

  def test_creeping_lead_is_followed_without_a_lurch_and_slam(self):
    # lead does 0.4 m/s pulses (1.2 s on, 1.2 s off): the classic queue creep that used to make the car surge and slam
    def lead_v(t):
      return 0.4 if int(t / 1.2) % 2 == 0 else 0.0
    r = run(0.0, STOP_GAP + 0.3, lead_v=lead_v, secs=12.0, gain=1.6, creep=0.4)
    # never demanded hard braking, and the gap stayed near the target the whole time
    self.assertGreater(r['targets'].min(), -0.9)
    self.assertLess(r['gaps'][40:].max() - r['gaps'][40:].min(), 1.2)
    self.assertLess(r['vs'].max(), 1.0)

  def test_hands_back_when_the_lead_pulls_away(self):
    gov = StopGapGovernor(DT)
    self.assertIsNotNone(gov.update(True, 0.5, True, 5.0, 0.3, -0.3))   # lead barely moving: governed
    self.assertIsNone(gov.update(True, 0.5, True, 5.0, 1.5, 0.4))       # lead clearly faster: pull-away, hand back
    self.assertFalse(gov.engaged)
    self.assertIsNone(gov.update(True, 1.0, True, 5.0, CREEP_LEAD_V + 0.5, 0.4))  # lead fast: MPC's job

  def test_only_ever_adds_braking_relative_to_the_mpc(self):
    gov = StopGapGovernor(DT)
    # inside the window with almost no brake needed, it must still not brake less than the MPC's -0.6
    a = gov.update(True, 0.5, True, STOP_GAP + 5.0, 0.0, -0.6)
    self.assertLessEqual(a, -0.6 + 1e-9)
    self.assertIsNone(gov.update(True, 0.5, True, STOP_GAP + 11.0, 0.0, -0.6))  # past S_ENGAGE: MPC keeps control

  def test_release_from_the_mpc_is_rate_limited(self):
    gov = StopGapGovernor(DT)
    # engage while the MPC was braking hard, then the MPC goes light: the governor eases off no faster than the rate
    gov.update(True, 0.4, True, STOP_GAP + 6.0, 0.0, -0.9)
    a1 = gov.update(True, 0.4, True, STOP_GAP + 6.0, 0.0, -0.1)
    self.assertAlmostEqual(a1, -0.9 + RELEASE_RATE * DT, places=6)

  def test_holds_a_firm_brake_below_v_hold_and_past_the_point(self):
    gov = StopGapGovernor(DT)
    for v, s in ((0.36, 0.3), (0.15, 0.1), (0.0, 0.05), (0.0, -0.1), (0.2, -0.2)):
      gov.reset()
      self.assertLessEqual(gov.update(True, v, True, STOP_GAP + s, 0.0, -1.0), -A_HOLD + 1e-9)

  def test_brakes_harder_when_fast_and_close(self):
    gov = StopGapGovernor(DT)
    self.assertLess(gov.update(True, 2.0, True, STOP_GAP + 1.0, 0.0, -1.0), -1.5)
    self.assertGreaterEqual(gov.update(True, 3.0, True, STOP_GAP - 1.0, 0.0, -2.0), A_NEG_MAX)

  def test_engage_window_with_hysteresis(self):
    gov = StopGapGovernor(DT)
    self.assertIsNone(gov.update(True, V_ENGAGE + 0.5, True, STOP_GAP + 5.0, 0.0, -0.5))
    self.assertIsNone(gov.update(True, 2.0, True, STOP_GAP + S_ENGAGE + 0.5, 0.0, -0.5))
    self.assertIsNotNone(gov.update(True, 2.0, True, STOP_GAP + 5.0, 0.0, -0.5))
    self.assertIsNotNone(gov.update(True, V_ENGAGE + 0.5, True, STOP_GAP + 5.0, 0.0, -0.5))
    self.assertIsNone(gov.update(True, V_ENGAGE + 1.5, True, STOP_GAP + 5.0, 0.0, -0.5))


if __name__ == "__main__":
  unittest.main()
