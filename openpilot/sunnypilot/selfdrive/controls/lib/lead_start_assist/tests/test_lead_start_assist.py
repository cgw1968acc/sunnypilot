import unittest

from openpilot.sunnypilot.selfdrive.controls.lib.lead_start_assist.lead_start_assist import (LeadStartAssist, ACTIVE_A_V, ACTIVE_TIMEOUT,
                                                                                            V_EGO_HANDOFF, GAP_MIN_ACTIVE)

DT = 0.05


def settle(assist, d_rel=6.0, frames=10):
  """Both cars stopped for half a second."""
  out = None
  for _ in range(frames):
    out = assist.update(True, 0.0, True, d_rel, 0.0, 0.0)
  return out


class TestLeadStartAssist(unittest.TestCase):
  def test_nothing_while_both_cars_sit_still(self):
    assist = LeadStartAssist(DT)
    self.assertIsNone(settle(assist))
    self.assertTrue(assist.armed)
    self.assertFalse(assist.active)

  def test_lead_speed_triggers_floor_within_two_frames(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    self.assertIsNone(assist.update(True, 0.0, True, 6.0, 0.6, 0.6))
    out = assist.update(True, 0.0, True, 6.02, 0.6, 0.6)
    self.assertAlmostEqual(out, ACTIVE_A_V[0])
    self.assertTrue(assist.active)

  def test_gap_growth_triggers_even_when_lead_speed_reads_low(self):
    assist = LeadStartAssist(DT)
    settle(assist, d_rel=5.5)
    self.assertIsNone(assist.update(True, 0.0, True, 5.85, 0.2, 0.2))
    self.assertAlmostEqual(assist.update(True, 0.0, True, 5.9, 0.2, 0.2), ACTIVE_A_V[0])

  def test_radar_jitter_does_not_trigger(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    for v in (0.5, 0.0, 0.5, 0.0, 0.1, 0.6, 0.0):
      self.assertIsNone(assist.update(True, 0.0, True, 6.0 + 0.05 * (v > 0), v, v))
    self.assertFalse(assist.active)

  def test_floor_ramps_up_and_hands_off_when_rolling(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    assist.update(True, 0.0, True, 6.0, 0.8, 0.8)
    assist.update(True, 0.0, True, 6.1, 0.8, 0.8)
    first = assist.update(True, 0.1, True, 6.2, 1.0, 0.9)
    later = None
    for _ in range(12):
      later = assist.update(True, 0.5, True, 7.0, 1.5, 1.0)
    self.assertLess(first, later)
    self.assertAlmostEqual(later, ACTIVE_A_V[1])
    self.assertIsNone(assist.update(True, V_EGO_HANDOFF + 0.1, True, 8.0, 2.0, 0.7))
    self.assertFalse(assist.active)

  def test_releases_when_lead_stops_again_and_ego_closes(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    assist.update(True, 0.0, True, 6.0, 0.8, 0.8)
    assist.update(True, 0.0, True, 6.1, 0.8, 0.8)
    self.assertIsNotNone(assist.update(True, 0.3, True, 6.5, 0.0, -0.1))  # lead just stopped, still within noise
    self.assertIsNone(assist.update(True, 0.6, True, 6.4, 0.0, -0.6))  # ego clearly closing, hand back to MPC
    self.assertFalse(assist.active)

  def test_releases_on_small_gap(self):
    assist = LeadStartAssist(DT)
    settle(assist, d_rel=4.0)
    assist.update(True, 0.0, True, 4.0, 0.8, 0.8)
    assist.update(True, 0.0, True, 4.1, 0.8, 0.8)
    self.assertIsNone(assist.update(True, 0.3, True, GAP_MIN_ACTIVE - 0.1, 0.8, 0.5))

  def test_times_out(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    assist.update(True, 0.0, True, 6.0, 0.8, 0.8)
    assist.update(True, 0.0, True, 6.1, 0.8, 0.8)
    out = ACTIVE_A_V[0]
    for _ in range(int(ACTIVE_TIMEOUT / DT) + 2):
      out = assist.update(True, 0.2, True, 8.0, 0.8, 0.6)
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

  def test_new_closer_lead_needs_to_settle_again(self):
    assist = LeadStartAssist(DT)
    settle(assist, d_rel=8.0)
    # a car cuts in at 4 m and moves off right away: not the car we were queued behind, no floor
    self.assertIsNone(assist.update(True, 0.0, True, 4.0, 1.0, 1.0))
    self.assertIsNone(assist.update(True, 0.0, True, 4.1, 1.0, 1.0))
    self.assertFalse(assist.active)
    self.assertFalse(assist.armed)
    # once it has settled in front of us it becomes the lead we follow
    settle(assist, d_rel=4.2)
    assist.update(True, 0.0, True, 4.2, 0.8, 0.8)
    self.assertAlmostEqual(assist.update(True, 0.0, True, 4.3, 0.8, 0.8), ACTIVE_A_V[0])

  def test_lead_rolling_back_slightly_is_not_a_cut_in(self):
    assist = LeadStartAssist(DT)
    settle(assist, d_rel=6.0)
    assist.update(True, 0.0, True, 5.7, 0.0, 0.0)  # lead creeps 30 cm closer (hill roll-back)
    self.assertTrue(assist.armed)
    assist.update(True, 0.0, True, 5.8, 0.8, 0.8)
    self.assertAlmostEqual(assist.update(True, 0.0, True, 5.9, 0.8, 0.8), ACTIVE_A_V[0])


if __name__ == "__main__":
  unittest.main()
