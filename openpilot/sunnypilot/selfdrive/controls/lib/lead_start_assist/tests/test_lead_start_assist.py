import unittest

from openpilot.sunnypilot.selfdrive.controls.lib.lead_start_assist.lead_start_assist import (LeadStartAssist, ACTIVE_A_V, ACTIVE_TIMEOUT,
                                                                                            V_EGO_HANDOFF, GAP_MIN_ACTIVE, GO_CONFIRM_TIME)

DT = 0.05
CONFIRM_FRAMES = round(GO_CONFIRM_TIME / DT)


def settle(assist, d_rel=6.0, frames=6):
  """Both cars stopped long enough to arm."""
  out = None
  for _ in range(frames):
    out = assist.update(True, 0.0, True, d_rel, 0.0, 0.0)
  return out


def depart(assist, d0=6.0, v_lead=0.6, frames=CONFIRM_FRAMES, v_ego=0.0):
  """Lead pulls away at v_lead (gap grows), ego still stopped. Returns the last output."""
  out = None
  d = d0
  for _ in range(frames):
    d += v_lead * DT
    out = assist.update(True, v_ego, True, d, v_lead, v_lead - v_ego)
  return out


class TestLeadStartAssist(unittest.TestCase):
  def test_nothing_while_both_cars_sit_still(self):
    assist = LeadStartAssist(DT)
    self.assertIsNone(settle(assist))
    self.assertTrue(assist.armed)
    self.assertFalse(assist.active)

  def test_sustained_lead_motion_triggers_the_floor(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    # not yet after a couple of frames...
    assist.update(True, 0.0, True, 6.03, 0.6, 0.6)
    self.assertFalse(assist.active)
    # ...but once the motion has held for GO_CONFIRM_TIME it fires at the gentle floor
    out = depart(assist, d0=6.03, v_lead=0.6)
    self.assertGreaterEqual(out, ACTIVE_A_V[0] - 1e-9)
    self.assertLess(out, ACTIVE_A_V[0] + 0.05)  # just above the floor as it starts ramping
    self.assertTrue(assist.active)

  def test_brief_speed_spike_on_a_still_lead_does_not_trigger(self):
    # the rlog false trigger (t=340.8): a still lead whose radar speed spikes to 0.25 for 1-2 frames must NOT push
    assist = LeadStartAssist(DT)
    settle(assist)
    for vl in (0.0, 0.25, 0.25, 0.0, -0.1, 0.0):  # spike shorter than GO_CONFIRM_TIME
      self.assertIsNone(assist.update(True, 0.0, True, 6.0, vl, vl))
    self.assertFalse(assist.active)

  def test_gap_growth_needs_real_lead_motion_to_trigger(self):
    assist = LeadStartAssist(DT)
    settle(assist, d_rel=5.5)
    # gap opens but lead speed ~0 (drift): never triggers however long
    d = 6.0
    for _ in range(CONFIRM_FRAMES + 3):
      d += 0.05
      self.assertIsNone(assist.update(True, 0.0, True, d, 0.05, 0.05))
    self.assertFalse(assist.active)

  def test_radar_jitter_does_not_trigger(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    for vl in (0.5, 0.0, 0.5, 0.0, 0.1, 0.6, 0.0, 0.5, 0.0):
      self.assertIsNone(assist.update(True, 0.0, True, 6.0, vl, vl))
    self.assertFalse(assist.active)

  def test_floor_ramps_up_and_hands_off_when_rolling(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    first = depart(assist, d0=6.0, v_lead=1.0)
    self.assertAlmostEqual(first, ACTIVE_A_V[0])
    later = None
    for _ in range(12):
      later = assist.update(True, 0.5, True, 8.0, 1.5, 1.0)
    self.assertLess(first, later)
    self.assertAlmostEqual(later, ACTIVE_A_V[1])
    self.assertIsNone(assist.update(True, V_EGO_HANDOFF + 0.1, True, 9.0, 2.0, 0.7))
    self.assertFalse(assist.active)

  def test_releases_when_ego_closes_on_a_lead_that_did_not_really_depart(self):
    # this is the rlog fix: once active, if the gap shrinks back (we pushed into a still lead), hand back at once
    assist = LeadStartAssist(DT)
    settle(assist, d_rel=6.0)
    self.assertIsNotNone(depart(assist, d0=6.0, v_lead=0.6))
    # ego now rolling and the gap has closed below where it armed -> release
    self.assertIsNone(assist.update(True, 0.4, True, 5.4, 0.0, -0.4))
    self.assertFalse(assist.active)

  def test_releases_on_small_gap(self):
    assist = LeadStartAssist(DT)
    settle(assist, d_rel=4.0)
    self.assertIsNotNone(depart(assist, d0=4.0, v_lead=0.6))
    self.assertIsNone(assist.update(True, 0.3, True, GAP_MIN_ACTIVE - 0.1, 0.8, 0.5))

  def test_times_out(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    self.assertIsNotNone(depart(assist, d0=6.0, v_lead=0.8))
    out = ACTIVE_A_V[0]
    d = 8.0
    for _ in range(int(ACTIVE_TIMEOUT / DT) + 2):
      d += 0.8 * DT
      out = assist.update(True, 0.6, True, d, 0.8, 0.2)
    self.assertIsNone(out)

  def test_not_allowed_or_no_lead_resets(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    self.assertIsNone(assist.update(False, 0.0, True, 6.0, 1.0, 1.0))
    self.assertFalse(assist.armed)
    settle(assist)
    self.assertIsNone(assist.update(True, 0.0, False, 6.0, 1.0, 1.0))
    self.assertFalse(assist.armed)

  def test_lead_outside_window_never_arms(self):
    assist = LeadStartAssist(DT)
    self.assertIsNone(settle(assist, d_rel=20.0))
    self.assertFalse(assist.armed)
    self.assertIsNone(settle(assist, d_rel=1.5))
    self.assertFalse(assist.armed)

  def test_lead_rolling_back_slightly_is_not_a_cut_in(self):
    assist = LeadStartAssist(DT)
    settle(assist, d_rel=6.0)
    assist.update(True, 0.0, True, 5.7, 0.0, 0.0)  # lead creeps 30 cm closer (hill roll-back)
    self.assertTrue(assist.armed)
    self.assertIsNotNone(depart(assist, d0=5.8, v_lead=0.8))


if __name__ == "__main__":
  unittest.main()
