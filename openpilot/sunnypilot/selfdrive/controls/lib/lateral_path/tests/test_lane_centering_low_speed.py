import unittest

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.lane_centering import LaneCentering
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.lane_centering_low_speed import (LaneCenteringLowSpeed,
                                                                                               LOW_CENTER_DEADBAND,
                                                                                               LOW_CENTER_MAX_LAT_ACCEL,
                                                                                               LOW_CENTER_JERK,
                                                                                               LOW_CENTER_LOST_HOLD_T)
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.tests.test_lane_centering import model

DT = 0.01
V50 = 50 * CV.KPH_TO_MS


def make():
  lc = LaneCentering(DT)
  return lc, LaneCenteringLowSpeed(DT, lc)


def step(lc, low, m, v, n, active=True, pressed=False, blinker=False):
  c = 0.0
  for _ in range(n):
    lc.update(m, v, active, pressed)  # measurement shared with the low-speed centering
    c = low.update(m, v, active, pressed, blinker)
  return c


class TestLaneCenteringLowSpeed(unittest.TestCase):
  def test_inside_the_dead_band_nothing(self):
    lc, low = make()
    self.assertEqual(step(lc, low, model(LOW_CENTER_DEADBAND - 0.05), V50, 300), 0.0)

  def test_car_right_of_centre_is_pulled_left_at_once_and_symmetric(self):
    lc, low = make()
    c = step(lc, low, model(-0.85), V50, 100)  # 1 s after the lines appear, 0.85 m right (route 140 23:41:41)
    self.assertLess(c, 0.0)  # left = negative curvature
    self.assertGreater(-c * V50 ** 2, 0.25)  # already a real correction (the old trim: still 0 after its 1 s engage)
    lc2, low2 = make()
    self.assertAlmostEqual(step(lc2, low2, model(0.85), V50, 100), -c, places=9)

  def test_capped_and_rate_limited(self):
    lc, low = make()
    prev = 0.0
    for _ in range(600):
      lc.update(model(-2.0), V50, True, False)
      c = low.update(model(-2.0), V50, True, False)
      self.assertLessEqual(abs(c - prev) * V50 ** 2, LOW_CENTER_JERK * DT + 1e-9)
      prev = c
    self.assertAlmostEqual(-prev * V50 ** 2, LOW_CENTER_MAX_LAT_ACCEL, places=6)

  def test_off_with_blinker_steering_inactive_and_on_the_highway(self):
    for kw in ({'blinker': True}, {'pressed': True}, {'active': False}):
      lc, low = make()
      self.assertEqual(step(lc, low, model(-0.85), V50, 300, **kw), 0.0, kw)
    lc, low = make()
    self.assertEqual(step(lc, low, model(-0.85), 85 * CV.KPH_TO_MS, 300), 0.0)  # highway: LaneCentering's job

  def test_lost_lines_hold_then_let_go(self):
    lc, low = make()
    c0 = step(lc, low, model(-0.85), V50, 300)
    weak = model(-0.85, probs=(0.2, 0.2))
    c_hold = step(lc, low, weak, V50, int(0.5 * LOW_CENTER_LOST_HOLD_T / DT))
    self.assertAlmostEqual(c_hold, c0, places=9)
    c_end = step(lc, low, weak, V50, int((LOW_CENTER_LOST_HOLD_T + 3.0) / DT))
    self.assertEqual(c_end, 0.0)


if __name__ == "__main__":
  unittest.main()
