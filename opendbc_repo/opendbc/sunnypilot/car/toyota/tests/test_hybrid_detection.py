import unittest

from opendbc.car import structs
from opendbc.car.toyota.interface import CarInterface
from opendbc.sunnypilot.car.toyota.fingerprints_ext import hybrid_can_messages
from opendbc.car.toyota.values import CAR, ToyotaFlags, Ecu

HYBRID_BUS0 = {0x127: 8, 0x245: 5, 0x1C4: 8, 0x3BC: 8}   # GEAR_PACKET_HYBRID, GAS_PEDAL_HYBRID, no GAS_PEDAL
PETROL_BUS0 = {0x2C1: 8, 0x1C4: 8, 0x3BC: 8}              # GAS_PEDAL, no hybrid messages
SECOC_PETROL_BUS0 = {0x127: 8, 0x2C1: 8, 0x3BC: 8}        # newer petrol cars also send 0x127 but keep GAS_PEDAL


def params(bus0, car_fw=()):
  fp = {0: dict(bus0), 1: {}, 2: {}}
  return CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS2, fp, list(car_fw), False, False, False)


class TestHybridDetection(unittest.TestCase):
  def test_rule(self):
    self.assertTrue(hybrid_can_messages({0: HYBRID_BUS0}))
    self.assertFalse(hybrid_can_messages({0: PETROL_BUS0}))
    self.assertFalse(hybrid_can_messages({0: SECOC_PETROL_BUS0}))
    self.assertFalse(hybrid_can_messages({0: {}}))
    self.assertFalse(hybrid_can_messages({}))

  def test_hybrid_from_can_without_hybrid_ecu(self):
    CP = params(HYBRID_BUS0)
    self.assertTrue(CP.flags & ToyotaFlags.HYBRID)
    self.assertAlmostEqual(CP.longitudinalActuatorDelay, 0.05)  # hybrid-gated tuning follows the flag
    self.assertAlmostEqual(CP.longitudinalActuatorDelay, 0.05)

  def test_petrol_stays_petrol(self):
    for bus0 in (PETROL_BUS0, SECOC_PETROL_BUS0, {}):
      with self.subTest(bus0=bus0):
        CP = params(bus0)
        self.assertFalse(CP.flags & ToyotaFlags.HYBRID)
        self.assertAlmostEqual(CP.stopAccel, -2.0)

  def test_hybrid_ecu_still_detects(self):
    fw = structs.CarParams.CarFw(ecu=Ecu.hybrid, address=0x7e2, fwVersion=b"x")
    CP = params(PETROL_BUS0, [fw])
    self.assertTrue(CP.flags & ToyotaFlags.HYBRID)


if __name__ == "__main__":
  unittest.main()
