import unittest

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.lane_centering import (LaneCentering, LaneTrimLowSpeed,
                                                                                     LANE_TRIM_ENGAGE_OFFSET,
                                                                                     LANE_TRIM_ENGAGE_TIME,
                                                                                     LANE_TRIM_MAX_LAT_ACCEL,
                                                                                     LANE_TRIM_FADE_BP)
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.tests.test_lane_centering import model

DT = 0.01


def make():
  lc = LaneCentering(DT)
  return lc, LaneTrimLowSpeed(DT, lc)


def step(lc, tr, y, v, n, k_model=0.0):
  c = 0.0
  for _ in range(n):
    lc.update(model(y), v, True, False)          # measurement shared with the trim
    c = tr.update(model(y), v, True, False, k_model)
  return c


class TestLaneTrimLowSpeed(unittest.TestCase):
  def test_sustained_offset_builds_a_small_trim_at_low_speed(self):
    v = 50 * CV.KPH_TO_MS
    lc, tr = make()
    c_early = step(lc, tr, LANE_TRIM_ENGAGE_OFFSET + 0.2, v, int(LANE_TRIM_ENGAGE_TIME * 0.5 / DT))
    self.assertEqual(c_early, 0.0)                # not yet: the offset must last ENGAGE_TIME first
    c = step(lc, tr, LANE_TRIM_ENGAGE_OFFSET + 0.2, v, int(6.0 / DT))
    self.assertGreater(c, 0.0)                    # car left of centre -> right (positive) curvature
    self.assertLessEqual(c * v ** 2, LANE_TRIM_MAX_LAT_ACCEL + 1e-6)

  def test_small_offset_is_left_alone_and_highway_speed_is_the_centering_s_job(self):
    lc, tr = make()
    self.assertEqual(step(lc, tr, 0.1, 50 * CV.KPH_TO_MS, int(6.0 / DT)), 0.0)
    lc, tr = make()
    self.assertEqual(step(lc, tr, 0.6, LANE_TRIM_FADE_BP[1] + 1.0, int(6.0 / DT)), 0.0)

  def test_releases_inside_the_band(self):
    v = 50 * CV.KPH_TO_MS
    lc, tr = make()
    c = step(lc, tr, 0.6, v, int(6.0 / DT))
    self.assertGreater(c, 0.0)
    c2 = step(lc, tr, 0.0, v, int(6.0 / DT))
    self.assertLess(c2, c * 0.2)


if __name__ == "__main__":
  unittest.main()
