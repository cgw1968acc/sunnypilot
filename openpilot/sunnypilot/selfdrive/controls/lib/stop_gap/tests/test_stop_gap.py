import unittest

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap import (StopGapGovernor, STOP_GAP, A_NEG_MAX, V_ENGAGE, S_ENGAGE,
                                                                          A_MIN, CREEP_LEAD_V, RELEASE_RATE)

DT = 0.05
STOP_ACCEL = -0.30          # hybrid stopAccel (Toyota interface)
STOPPING_DECEL_RATE = 0.3   # longcontrol ramp toward stopAccel


def mpc_like(v_close, s):
  return -float(np.clip(v_close * v_close / (2.0 * max(s, 0.5)), 0.3, 1.2)) if v_close > 0 else -0.05


def run(v0, gap0, lead_v=None, secs=16.0, lag=0.4, gain=1.0, creep=0.0):
  """Closed loop against a first-order lagged plant that brakes `gain` times the request below 1.5 m/s and creeps at
  +creep when the *delivered* brake is lighter than the creep. longcontrol's stopping state is modelled: once
  should_stop (v < 0.25 and target < 0.1) it holds the last output and ramps it toward STOP_ACCEL, ignoring the target."""
  if lead_v is None:
    lead_v = lambda t: 0.0  # noqa: E731
  gov = StopGapGovernor(DT)
  v, gap = v0, gap0
  a = mpc_like(v0 - lead_v(0.0), gap0 - STOP_GAP)
  out = a
  targets, gaps, vs, outs = [], [], [], []
  t = 0.0
  t_stop = None
  hold = None
  crept = False
  gap_stop = None
  stopping = False
  while t < secs:
    vl = max(lead_v(t), 0.0)
    target = gov.update(True, v, True, gap, vl, mpc_like(v - vl, gap - STOP_GAP))
    if target is None:
      target = mpc_like(v - vl, gap - STOP_GAP)
    # longcontrol state machine
    if stopping:
      if out > STOP_ACCEL:
        out = min(out, 0.0) - STOPPING_DECEL_RATE * DT
      # exit stopping only if a clear go (lead moving) — here leads stay stopped in the stop tests
      if vl > 0.2:
        stopping = False
    else:
      out = target
      if v < 0.25 and target < 0.1 and vl < 0.2:
        stopping = True
        if t_stop is None:
          t_stop, gap_stop = t, gap
    targets.append(target)
    outs.append(out)
    gaps.append(gap)
    vs.append(v)
    # plant: PCM delivers `gain`x the (braking) command below 1.5 m/s; creep torque pushes when delivered brake is light
    delivered = out * gain if (out < 0 and v < 1.5) else out
    net = delivered + (creep if delivered > -creep else 0.0)
    a += (net - a) * DT / lag
    v = max(v + a * DT, 0.0)
    gap += (vl - v) * DT
    t += DT
    if t_stop is not None and t > t_stop + 0.5 and v > 0.1 and vl < 0.05:
      crept = True
    if t_stop is not None and t > t_stop + 3.0 and vl < 0.05:
      hold = out
      break
  return {"gap": gap_stop if gap_stop is not None else gap, "hold": hold, "crept": crept,
          "targets": np.array(targets), "gaps": np.array(gaps), "vs": np.array(vs), "outs": np.array(outs)}


class TestStopGapGovernor(unittest.TestCase):
  def test_stops_near_the_gap_and_holds_firm_without_creeping(self):
    for v0, gap0 in ((2.5, 8.5), (3.0, 7.5), (2.0, 6.5), (1.2, 5.0)):
      r = run(v0, gap0, gain=1.6, creep=0.06)
      self.assertAlmostEqual(r['gap'], STOP_GAP, delta=0.6, msg=f'{v0=} {gap0=}')
      self.assertFalse(r['crept'], msg=f'{v0=} {gap0=}')          # firm hold, no roll-on after the stop
      self.assertLessEqual(r['hold'], STOP_ACCEL + 0.05, msg=f'{v0=} {gap0=}')

  def test_final_metres_glide_in_gently(self):
    # approaching a stopped lead, the brake target tapers (no forced firm clamp): the last second is light
    r = run(2.0, 6.5)
    near = r['targets'][(r['vs'] < 0.8) & (r['vs'] > 0.15)]
    self.assertTrue(len(near) > 3)
    self.assertGreater(near.min(), -0.6)   # nothing harsh in the glide
    self.assertLess(near.max(), 0.0)       # but always a (light) brake, never a release to zero

  def test_creeping_lead_is_followed_without_a_lurch_and_slam(self):
    def lead_v(t):
      return 0.4 if int(t / 1.2) % 2 == 0 else 0.0
    r = run(0.0, STOP_GAP + 0.3, lead_v=lead_v, secs=12.0, gain=1.6, creep=0.06)
    self.assertGreater(r['targets'].min(), -0.9)
    self.assertLess(r['vs'].max(), 1.0)

  def test_hands_back_when_the_lead_pulls_away(self):
    gov = StopGapGovernor(DT)
    self.assertIsNotNone(gov.update(True, 0.5, True, 5.0, 0.3, -0.3))
    self.assertIsNone(gov.update(True, 0.5, True, 5.0, 1.5, 0.4))
    self.assertFalse(gov.engaged)
    self.assertIsNone(gov.update(True, 1.0, True, 5.0, CREEP_LEAD_V + 0.5, 0.4))

  def test_only_ever_adds_braking_relative_to_the_mpc(self):
    gov = StopGapGovernor(DT)
    a = gov.update(True, 0.5, True, STOP_GAP + 5.0, 0.0, -0.6)
    self.assertLessEqual(a, -0.6 + 1e-9)
    self.assertIsNone(gov.update(True, 0.5, True, STOP_GAP + 11.0, 0.0, -0.6))

  def test_release_from_the_mpc_is_rate_limited(self):
    gov = StopGapGovernor(DT)
    gov.update(True, 0.4, True, STOP_GAP + 6.0, 0.0, -0.9)
    a1 = gov.update(True, 0.4, True, STOP_GAP + 6.0, 0.0, -0.1)
    self.assertAlmostEqual(a1, -0.9 + RELEASE_RATE * DT, places=6)

  def test_glide_floor_keeps_a_light_brake_near_the_stop(self):
    gov = StopGapGovernor(DT)
    a = gov.update(True, 0.2, True, STOP_GAP + 0.3, 0.0, -0.05)
    self.assertLessEqual(a, -A_MIN + 1e-9)   # at least the gentle glide floor
    self.assertGreater(a, -0.4)              # but not a firm clamp

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
