import unittest

from openpilot.sunnypilot.selfdrive.controls.lib.lead_start_assist.lead_start_assist import (
  LeadStartAssist, ARM_TIME, FLOOR_MIN, T_MATCH, ACTIVE_TIMEOUT, GAP_MIN_ACTIVE, LEAD_D_MIN)

DT = 0.05


def settle(assist, d_rel=3.0, frames=10, v_lead=0.0):
  out = None
  for _ in range(frames):
    out = assist.update(True, 0.0, True, d_rel, v_lead, v_lead)
  return out


def depart(assist, d0=3.0, v_lead=0.6, frames=10, v_ego=0.0, a_max=1.65):
  """Lead pulls away at v_lead (gap grows), ego still stopped. Returns the first non-None output (or None)."""
  d = d0
  for _ in range(frames):
    d += (v_lead - v_ego) * DT
    out = assist.update(True, v_ego, True, d, v_lead, v_lead - v_ego, a_max=a_max)
    if out is not None:
      return out
  return None


class TestLeadStartAssist(unittest.TestCase):
  def test_arms_while_ego_stands_even_with_a_noisy_lead_speed(self):
    # route 00000093 22:20:58: a stopped lead whose radar speed sits at 0.2-0.4 m/s must not keep it from arming
    assist = LeadStartAssist(DT)
    self.assertIsNone(settle(assist, v_lead=0.3, frames=round(ARM_TIME / DT) + 1))
    self.assertTrue(assist.armed)

  def test_noisy_speed_on_a_still_lead_never_launches(self):
    # route 00000093 22:20:12.8: lead speed 0.15-0.30 m/s, gap flat at 3.0 +- 0.1 m -> the old trigger launched the car
    assist = LeadStartAssist(DT)
    settle(assist)
    for k in range(200):
      vl = (0.15, 0.22, 0.30, 0.28, 0.25, 0.20)[k % 6]
      d = 3.0 + (0.1, -0.1, 0.05, 0.0, -0.05, 0.08)[k % 6]
      self.assertIsNone(assist.update(True, 0.0, True, d, vl, vl, a_max=1.65))
    self.assertFalse(assist.active)

  def test_a_real_departure_fires_within_half_a_second(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    self.assertIsNotNone(depart(assist, v_lead=0.6, frames=10))
    self.assertTrue(assist.active)

  def test_floor_follows_the_speed_difference_up_to_the_profile_max(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    out = depart(assist, v_lead=0.6)
    self.assertAlmostEqual(out, max(FLOOR_MIN, 0.6 / T_MATCH), places=6)
    # the lead speeds up: the floor reaches the accel profile max, never above
    out = assist.update(True, 0.0, True, 4.0, 3.0, 3.0, a_max=1.65)
    self.assertAlmostEqual(out, 1.65, places=6)
    # a creeping lead is followed gently
    assist2 = LeadStartAssist(DT)
    settle(assist2)
    out = depart(assist2, v_lead=0.25, frames=40)
    self.assertAlmostEqual(out, FLOOR_MIN, places=6)

  def test_hands_back_once_ego_has_the_lead_speed(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    depart(assist, v_lead=1.0)
    self.assertIsNotNone(assist.update(True, 0.5, True, 4.0, 1.0, 0.5, a_max=1.65))
    self.assertIsNone(assist.update(True, 1.0, True, 4.0, 1.0, 0.0, a_max=1.65))
    self.assertFalse(assist.active)

  def test_releases_on_small_gap_closing_and_timeout(self):
    for d, vl, ve in ((GAP_MIN_ACTIVE - 0.1, 1.0, 0.2), (3.5, 0.5, 0.9)):
      assist = LeadStartAssist(DT)
      settle(assist)
      depart(assist, v_lead=1.0)
      self.assertIsNone(assist.update(True, ve, True, d, vl, vl - ve, a_max=1.65))
    assist = LeadStartAssist(DT)
    settle(assist)
    depart(assist, v_lead=1.0)
    out = 0
    d = 3.5
    for _ in range(round(ACTIVE_TIMEOUT / DT) + 2):
      d += 1.0 * DT
      out = assist.update(True, 0.1, True, d, 1.0, 0.9, a_max=1.65)
    self.assertIsNone(out)

  def test_lead_too_close_cut_in_or_not_allowed(self):
    assist = LeadStartAssist(DT)
    for _ in range(40):  # a lead closer than LEAD_D_MIN: never, whatever its radar speed says
      self.assertIsNone(assist.update(True, 0.0, True, LEAD_D_MIN - 0.1, 1.0, 1.0, a_max=1.65))
    assist = LeadStartAssist(DT)
    settle(assist, d_rel=6.0)
    self.assertIsNone(assist.update(True, 0.0, True, 4.5, 0.0, 0.0))  # cut-in: gap jumps 1.5 m closer
    self.assertFalse(assist.armed)
    assist = LeadStartAssist(DT)
    settle(assist)
    d = 3.0
    for _ in range(10):
      d += 0.05
      self.assertIsNone(assist.update(False, 0.0, True, d, 1.0, 1.0, a_max=1.65))

  def test_rolling_ego_disarms(self):
    assist = LeadStartAssist(DT)
    settle(assist)
    assist.update(True, 0.5, True, 3.0, 0.0, -0.5)
    self.assertFalse(assist.armed)


if __name__ == "__main__":
  unittest.main()
