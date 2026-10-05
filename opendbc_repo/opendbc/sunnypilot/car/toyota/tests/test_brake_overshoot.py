import unittest

from opendbc.sunnypilot.car.toyota.brake_onset import (BrakeOvershootLimiter, OVERSHOOT_CMD_CEIL, OVERSHOOT_DEADBAND,
                                                       OVERSHOOT_MAX)

DT = 0.03


class TestBrakeOvershootLimiter(unittest.TestCase):
  def run_steady(self, req, a_ego, n=200, active=True):
    lim = BrakeOvershootLimiter(DT)
    for _ in range(n):
      lim.update(req, a_ego, active)
    return lim

  def test_moderate_braking_is_untouched(self):
    self.assertEqual(self.run_steady(-2.0, -3.0).lift, 0.0)

  def test_overshoot_inside_the_deadband_is_untouched(self):
    self.assertEqual(self.run_steady(-3.4, -3.4 - OVERSHOOT_DEADBAND + 0.01).lift, 0.0)

  def test_19_41_overshoot_is_taken_back(self):
    lim = self.run_steady(-3.4, -4.3)
    self.assertAlmostEqual(lim.lift, 0.8 * (0.9 - OVERSHOOT_DEADBAND), places=6)
    self.assertAlmostEqual(lim.apply(-3.4), -3.4 + lim.lift, places=6)

  def test_under_braking_never_lifts(self):
    self.assertEqual(self.run_steady(-3.4, -2.5).lift, 0.0)

  def test_lift_is_capped_and_command_stays_a_firm_brake(self):
    lim = self.run_steady(-2.6, -6.0)
    self.assertEqual(lim.lift, OVERSHOOT_MAX)
    self.assertAlmostEqual(lim.apply(-2.6), -2.6 + OVERSHOOT_MAX)
    self.assertEqual(lim.apply(-2.2), OVERSHOOT_CMD_CEIL)   # a PID-lightened command is not lifted past a firm brake

  def test_lift_grows_and_fades_gradually(self):
    lim = BrakeOvershootLimiter(DT)
    prev = 0.0
    for _ in range(10):
      lift = lim.update(-3.4, -5.0)
      self.assertLessEqual(lift - prev, 3.0 * DT + 1e-9)
      prev = lift
    for _ in range(10):
      lift = lim.update(-1.0, -1.0)   # the hard request ends
      self.assertLessEqual(prev - lift, 2.0 * DT + 1e-9)
      prev = lift

  def test_inactive_gives_it_back(self):
    lim = self.run_steady(-3.4, -4.3)
    for _ in range(100):
      lim.update(-3.4, -4.3, active=False)
    self.assertEqual(lim.lift, 0.0)


if __name__ == "__main__":
  unittest.main()


class TestBrakeSpeedGain(unittest.TestCase):
  def test_gas_untouched(self):
    from opendbc.sunnypilot.car.toyota.brake_onset import brake_speed_gain
    self.assertEqual(brake_speed_gain(0.8, 12 / 3.6), 0.8)

  def test_end_of_stop_untouched_below_7kph(self):
    from opendbc.sunnypilot.car.toyota.brake_onset import brake_speed_gain
    for v in (0.0, 3 / 3.6, 7 / 3.6):
      self.assertEqual(brake_speed_gain(-0.6, v), -0.6)

  def test_handover_band_firmer_and_mid_speed_lighter(self):
    from opendbc.sunnypilot.car.toyota.brake_onset import brake_speed_gain
    self.assertAlmostEqual(brake_speed_gain(-1.0, 12 / 3.6), -1.3)
    self.assertAlmostEqual(brake_speed_gain(-1.0, 35 / 3.6), -0.9)
    self.assertAlmostEqual(brake_speed_gain(-1.0, 110 / 3.6), -1.0)
