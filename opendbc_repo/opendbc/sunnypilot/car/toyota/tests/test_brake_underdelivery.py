import unittest

from opendbc.car import structs
from opendbc.car.toyota.values import CAR, ToyotaFlags
from opendbc.sunnypilot.car.toyota.brake_onset import BrakeUnderdeliveryComp, is_altis_hybrid, UNDER_MAX

DT = 0.03


def make_cp(fingerprint, hybrid, fws):
  cp = structs.CarParams()
  cp.carFingerprint = fingerprint
  cp.flags = ToyotaFlags.HYBRID.value if hybrid else 0
  cp.carFw = [structs.CarParams.CarFw(ecu=e, fwVersion=v) for e, v in fws]
  return cp


ALTIS_FW = [("eps", b'\x018965B12470\x00'), ("abs", b'F152612800\x00'), ("hybrid", b'\x02899831220000\x00')]
CROSS_FW = [("eps", b'8965B16210\x00'), ("abs", b'F152616070\x00'), ("hybrid", b'\x02899831636000\x00')]


class TestAltisDetection(unittest.TestCase):
  def test_altis_hybrid(self):
    self.assertTrue(is_altis_hybrid(make_cp(CAR.TOYOTA_COROLLA_TSS2, True, ALTIS_FW)))

  def test_altis_with_only_one_fw_answer(self):
    self.assertTrue(is_altis_hybrid(make_cp(CAR.TOYOTA_COROLLA_TSS2, True, ALTIS_FW[1:2])))

  def test_cross_is_excluded(self):
    self.assertFalse(is_altis_hybrid(make_cp(CAR.TOYOTA_COROLLA_TSS2, True, CROSS_FW)))

  def test_petrol_altis_and_other_cars_excluded(self):
    self.assertFalse(is_altis_hybrid(make_cp(CAR.TOYOTA_COROLLA_TSS2, False, ALTIS_FW)))
    self.assertFalse(is_altis_hybrid(make_cp(CAR.TOYOTA_RAV4_TSS2, True, ALTIS_FW)))

  def test_no_fw_answer_means_off(self):
    self.assertFalse(is_altis_hybrid(make_cp(CAR.TOYOTA_COROLLA_TSS2, True, [])))


class TestUnderdelivery(unittest.TestCase):
  def run_steady(self, req, a_ego, v_kph, n=200, enabled=True):
    c = BrakeUnderdeliveryComp(DT, enabled)
    for _ in range(n):
      c.update(req, a_ego, v_kph / 3.6)
    return c

  def test_23_55_00_shortfall_is_added_back(self):
    c = self.run_steady(-1.0, -0.54, 9.0)   # asked -1.0, car gave -0.54 at 9 km/h
    self.assertAlmostEqual(c.extra, 0.8 * (0.46 - 0.2), places=6)
    self.assertAlmostEqual(c.apply(-1.0), -1.0 - c.extra, places=9)

  def test_silent_when_the_car_delivers(self):
    self.assertEqual(self.run_steady(-1.8, -2.0, 20.0).extra, 0.0)
    self.assertEqual(self.run_steady(-1.0, -0.9, 12.0).extra, 0.0)

  def test_only_in_the_low_speed_band(self):
    self.assertEqual(self.run_steady(-1.0, -0.3, 30.0).extra, 0.0)
    self.assertEqual(self.run_steady(-0.6, -0.1, 4.0).extra, 0.0)

  def test_capped_and_gradual(self):
    c = BrakeUnderdeliveryComp(DT, True)
    prev = 0.0
    for _ in range(300):
      e = c.update(-2.0, 0.0, 12 / 3.6)
      self.assertLessEqual(e - prev, 1.0 * DT + 1e-9)
      prev = e
    self.assertEqual(c.extra, UNDER_MAX)

  def test_disabled_does_nothing(self):
    self.assertEqual(self.run_steady(-1.0, -0.3, 10.0, enabled=False).extra, 0.0)


if __name__ == "__main__":
  unittest.main()
