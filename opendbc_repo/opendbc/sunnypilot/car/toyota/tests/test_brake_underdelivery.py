import unittest
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.toyota.values import CAR, ToyotaFlags
from opendbc.sunnypilot.car.toyota.brake_onset import (BrakeHandoverFeedforward, is_altis_hybrid, HANDOVER_EXTRA_MAX,
                                                       HANDOVER_EXTRA_FRAC, HANDOVER_RATE, PID_BLEED_NEG, PID_BLEED_POS)

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


class TestHandoverFeedforward(unittest.TestCase):
  def run_steady(self, req, v_kph, n=200, enabled=True):
    c = BrakeHandoverFeedforward(DT, enabled)
    for _ in range(n):
      c.update(req, v_kph / 3.6)
    return c

  def test_full_extra_in_the_dip(self):
    c = self.run_steady(-0.84, 12.0)   # route 0000010e: asked -0.84 at 12-9 km/h, the car gave -0.61
    self.assertAlmostEqual(c.extra, HANDOVER_EXTRA_MAX, places=9)
    self.assertAlmostEqual(c.apply(-0.84), -0.84 - HANDOVER_EXTRA_MAX, places=9)

  def test_lighter_request_gets_a_quarter(self):
    self.assertAlmostEqual(self.run_steady(-0.5, 12.0).extra, HANDOVER_EXTRA_FRAC * 0.5, places=9)

  def test_fades_at_the_edges(self):
    self.assertAlmostEqual(self.run_steady(-1.0, 8.0).extra, 0.5 * HANDOVER_EXTRA_MAX, places=6)
    self.assertEqual(self.run_steady(-1.0, 5.0).extra, 0.0)
    self.assertEqual(self.run_steady(-1.0, 20.0).extra, 0.0)
    self.assertEqual(self.run_steady(-0.25, 12.0).extra, 0.0)

  def test_gradual(self):
    c = BrakeHandoverFeedforward(DT, True)
    prev = 0.0
    for _ in range(50):
      e = c.update(-1.0, 12 / 3.6)
      self.assertLessEqual(e - prev, HANDOVER_RATE * DT + 1e-9)
      prev = e

  def test_disabled_does_nothing(self):
    c = self.run_steady(-1.0, 12.0, enabled=False)
    self.assertEqual(c.extra, 0.0)
    pid = SimpleNamespace(i=-0.2)
    self.assertFalse(c.condition_pid(pid, -0.5, 1.0))
    self.assertEqual(pid.i, -0.2)


class TestPidHold(unittest.TestCase):
  def test_braking_integral_bleeds_and_freezes_below_9kph(self):
    c = BrakeHandoverFeedforward(DT, True)
    pid = SimpleNamespace(i=-0.15)
    self.assertTrue(c.condition_pid(pid, -0.6, 5 / 3.6))
    self.assertAlmostEqual(pid.i, -0.15 + PID_BLEED_NEG * DT, places=9)
    for _ in range(100):
      c.condition_pid(pid, -0.6, 5 / 3.6)
    self.assertEqual(pid.i, 0.0)

  def test_gas_integral_left_from_a_launch_bleeds_fast(self):
    c = BrakeHandoverFeedforward(DT, True)
    pid = SimpleNamespace(i=0.3)
    c.condition_pid(pid, -0.2, 3 / 3.6)
    self.assertAlmostEqual(pid.i, 0.3 - PID_BLEED_POS * DT, places=9)

  def test_untouched_above_9kph_or_when_not_braking(self):
    c = BrakeHandoverFeedforward(DT, True)
    pid = SimpleNamespace(i=-0.15)
    self.assertFalse(c.condition_pid(pid, -0.6, 12 / 3.6))
    self.assertFalse(c.condition_pid(pid, 0.3, 3 / 3.6))
    self.assertEqual(pid.i, -0.15)


if __name__ == "__main__":
  unittest.main()
