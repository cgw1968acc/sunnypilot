import unittest

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap import (StopGapGovernor, STOP_GAP, V_ENGAGE,
                                                                          CREEP_LEAD_V, RELEASE_RATE)

DT = 0.05
STOP_ACCEL = -1.0          # hybrid stopAccel (Toyota interface)
STOPPING_DECEL_RATE = 0.3   # longcontrol ramp toward stopAccel


def mpc_like(v_close, s):
  return -float(np.clip(v_close * v_close / (2.0 * max(s, 0.5)), 0.3, 1.2)) if v_close > 0 else -0.05


def run(v0, gap0, lead_v=None, secs=22.0, lag=0.4, gain=1.6, creep=0.06):
  """Closed loop against a lagged plant that brakes `gain`x the request below 1.5 m/s and creeps at +creep when the
  delivered brake is lighter than the creep torque. longcontrol's stopping state is modelled: it latches on the
  governor's should_stop, then holds the last output and ramps it toward STOP_ACCEL."""
  if lead_v is None:
    lead_v = lambda t: 0.0  # noqa: E731
  gov = StopGapGovernor(DT)
  v, gap = v0, gap0
  a = mpc_like(v0 - lead_v(0.0), gap0 - STOP_GAP)
  out = a
  targets, gaps, vs = [], [], []
  t = 0.0
  t_stop = None
  gap_stop = None
  hold = None
  crept = False
  stopping = False
  while t < secs:
    vl = max(lead_v(t), 0.0)
    res = gov.update(True, v, True, gap, vl, mpc_like(v - vl, gap - STOP_GAP))
    if res is None:
      target, sg = mpc_like(v - vl, gap - STOP_GAP), (v < 0.25 and vl < 0.2)
    else:
      target, sg = res
    if stopping:
      if out > STOP_ACCEL:
        out = min(out, 0.0) - STOPPING_DECEL_RATE * DT
      if vl > 0.3:
        stopping = False
    else:
      out = target
      if sg and v < 0.5:
        stopping = True
        if t_stop is None:
          t_stop, gap_stop = t, gap
    targets.append(target)
    gaps.append(gap)
    vs.append(v)
    delivered = out * gain if (out < 0 and v < 1.5) else out
    net = delivered + (creep if delivered > -creep else 0.0)
    a += (net - a) * DT / lag
    v = max(v + a * DT, 0.0)
    gap += (vl - v) * DT
    t += DT
    if t_stop is not None and t > t_stop + 0.4 and abs(v) > 0.1 and vl < 0.05:
      crept = True
    if t_stop is not None and t > t_stop + 3.0 and vl < 0.05:
      hold = out
      break
  return {"gap": gap_stop if gap_stop is not None else gap, "hold": hold, "crept": crept,
          "targets": np.array(targets), "gaps": np.array(gaps), "vs": np.array(vs)}


class TestStopGapGovernor(unittest.TestCase):
  def test_faster_and_slower_approaches_all_land_near_the_gap_without_stopping_short(self):
    for v0 in (8.0, 6.0, 4.0, 3.0):
      gap0 = STOP_GAP + v0 ** 2 / (2.0 * 1.0) + 2.0   # lead just inside the engage window
      r = run(v0, gap0)
      # lands close to 3 m and never stops short by a lot (which would then need a roll-forward)
      self.assertGreater(r['gap'], STOP_GAP - 0.5, msg=f'{v0=} too close at {r["gap"]:.2f}')
      self.assertLess(r['gap'], STOP_GAP + 0.8, msg=f'{v0=} stopped short at {r["gap"]:.2f}')
      self.assertFalse(r['crept'], msg=f'{v0=} rolled after stopping')
      self.assertLessEqual(r['hold'], STOP_ACCEL + 0.05, msg=f'{v0=} hold {r["hold"]}')

  def test_faster_approach_brakes_harder(self):
    r8 = run(8.0, STOP_GAP + 8.0 ** 2 / 2.0 + 2.0)
    r4 = run(4.0, STOP_GAP + 4.0 ** 2 / 2.0 + 2.0)
    self.assertLess(r8['targets'].min(), r4['targets'].min())   # more braking when entering faster

  def test_last_metre_glides_in(self):
    r = run(5.0, STOP_GAP + 5.0 ** 2 / 2.0 + 2.0)
    near = r['targets'][(r['vs'] < 0.8) & (r['vs'] > 0.15)]
    self.assertTrue(len(near) > 2)
    self.assertGreater(near.min(), -0.6)   # the glide is light, no firm grab at the end

  def test_creeping_lead_is_followed_without_a_lurch(self):
    def lead_v(t):
      return 0.4 if int(t / 1.2) % 2 == 0 else 0.0
    r = run(0.0, STOP_GAP + 0.3, lead_v=lead_v, secs=12.0)
    self.assertGreater(r['targets'].min(), -0.9)
    self.assertLess(r['vs'].max(), 1.0)

  def test_should_stop_latches_when_nearly_stopped(self):
    gov = StopGapGovernor(DT)
    _, sg_moving = gov.update(True, 1.0, True, STOP_GAP + 0.5, 0.0, -0.5)
    self.assertFalse(sg_moving)
    _, sg_stopped = gov.update(True, 0.15, True, STOP_GAP + 0.3, 0.0, -0.3)
    self.assertTrue(sg_stopped)

  def test_hands_back_when_the_lead_pulls_away(self):
    gov = StopGapGovernor(DT)
    self.assertIsNotNone(gov.update(True, 0.5, True, 5.0, 0.3, -0.3))
    self.assertIsNone(gov.update(True, 0.5, True, 5.0, 1.5, 0.4))
    self.assertIsNone(gov.update(True, 1.0, True, 5.0, CREEP_LEAD_V + 0.5, 0.4))

  def test_only_ever_adds_braking_relative_to_the_mpc(self):
    gov = StopGapGovernor(DT)
    a, _ = gov.update(True, 0.5, True, STOP_GAP + 2.5, 0.0, -0.9)
    self.assertLessEqual(a, -0.9 + RELEASE_RATE * DT + 1e-9)

  def test_release_from_the_mpc_is_rate_limited(self):
    gov = StopGapGovernor(DT)
    gov.update(True, 0.4, True, STOP_GAP + 2.5, 0.0, -0.9)
    a1, _ = gov.update(True, 0.4, True, STOP_GAP + 2.5, 0.0, -0.1)
    self.assertAlmostEqual(a1, -0.9 + RELEASE_RATE * DT, places=6)

  def test_engages_earlier_at_higher_speed(self):
    gov = StopGapGovernor(DT)
    self.assertIsNone(gov.update(True, V_ENGAGE + 1.5, True, STOP_GAP + 5.0, 0.0, -0.5))
    self.assertIsNotNone(gov.update(True, 6.0, True, STOP_GAP + 18.0, 0.0, -0.5))
    gov.reset()
    self.assertIsNone(gov.update(True, 6.0, True, STOP_GAP + 50.0, 0.0, -0.5))


if __name__ == "__main__":
  unittest.main()
