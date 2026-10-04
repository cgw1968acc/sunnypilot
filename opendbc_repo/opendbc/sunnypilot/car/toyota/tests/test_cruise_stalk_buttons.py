import inspect
import unittest

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.toyota.carstate import CarState, CruiseButton
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR

ButtonType = structs.CarState.ButtonEvent.Type

# Corolla Cross Hybrid TSS2 bus 0: GEAR_PACKET_HYBRID, GAS_PEDAL_HYBRID (hybrid rule), no GAS_PEDAL
HYBRID_BUS0 = {0x127: 8, 0x245: 5, 0x1C4: 8, 0x3BC: 8}


class FakeParser:
  def __init__(self, res: list[int], set_: list[int]):
    self.vl_all = {"CLUTCH": {"CRUISE_BTN_RES": res, "CRUISE_BTN_SET": set_}}


def make_cs():
  cs = CarState.__new__(CarState)  # skip the heavy __init__, only the button state is needed
  cs.cruise_button = CruiseButton.none
  return cs


class TestCruiseStalkButtons(unittest.TestCase):
  def setUp(self):
    self.cs = make_cs()

  def events(self, res, set_):
    return [(e.type, e.pressed) for e in self.cs.update_cruise_button_events(FakeParser(res, set_))]

  def test_short_res_press_gives_one_press_and_release(self):
    # ~0.5 s physical press = 7-8 genuine frames, spread over several 100 Hz iterations
    self.assertEqual(self.events([1], [0]), [(ButtonType.accelCruise, True)])
    for _ in range(6):
      self.assertEqual(self.events([1], [0]), [])
    self.assertEqual(self.events([0], [0]), [(ButtonType.accelCruise, False)])
    self.assertEqual(self.events([], []), [])

  def test_set_press(self):
    self.assertEqual(self.events([0], [1]), [(ButtonType.decelCruise, True)])
    self.assertEqual(self.events([0], [0]), [(ButtonType.decelCruise, False)])

  def test_press_and_release_in_one_iteration(self):
    self.assertEqual(self.events([1, 0], [0, 0]), [(ButtonType.accelCruise, True), (ButtonType.accelCruise, False)])

  def test_both_bits_is_not_a_press(self):
    self.assertEqual(self.events([1], [1]), [])
    self.assertEqual(self.events([0], [0]), [])

  def test_switching_buttons_releases_the_first(self):
    self.assertEqual(self.events([1], [0]), [(ButtonType.accelCruise, True)])
    self.assertEqual(self.events([0], [1]), [(ButtonType.accelCruise, False), (ButtonType.decelCruise, True)])

  def test_parser_registers_clutch_only_for_software_set_speed(self):
    src = inspect.getsource(CarState.get_can_parsers)
    self.assertIn('if not CP_SP.pcmCruiseSpeed:', src)
    self.assertIn('("CLUTCH", 15)', src)

  def test_clutch_stalk_bits_decode(self):
    # byte 0 of 0x361: bit 3 CANCEL, bit 4 SET/-, bit 5 RES/+ (generator _toyota_2017.dbc, built at runtime)
    cp = CANParser("toyota_new_mc_pt_generated", [("CLUTCH", 15)], 0)
    for byte0, expected in ((0x08, (1, 0, 0)), (0x10, (0, 1, 0)), (0x20, (0, 0, 1)), (0x00, (0, 0, 0))):
      cp.update([0, [(0x361, bytes([byte0, 0, 0, 0, 0, 0, 0, 0]), 0)]])
      got = tuple(int(cp.vl["CLUTCH"][k]) for k in ("CRUISE_BTN_CANCEL", "CRUISE_BTN_SET", "CRUISE_BTN_RES"))
      self.assertEqual(got, expected, hex(byte0))
    self.assertEqual(cp.vl_all["CLUTCH"]["CRUISE_BTN_RES"], [0.0])


class TestSoftwareSetSpeedPlatformFlag(unittest.TestCase):
  def test_corolla_cross_owns_the_set_speed(self):
    CP = get_params(CAR.TOYOTA_COROLLA_TSS2)
    self.assertTrue(CP.openpilotLongitudinalControl)
    self.assertTrue(CP.pcmCruise)  # the PCM still owns engagement
    self.assertFalse(get_params_sp(CAR.TOYOTA_COROLLA_TSS2).pcmCruiseSpeed)

  def test_other_toyotas_keep_the_pcm_set_speed(self):
    # a TSS2 car with openpilot longitudinal that is not the Corolla Cross
    CP_SP = get_params_sp(CAR.TOYOTA_RAV4_TSS2)
    self.assertTrue(CP_SP.pcmCruiseSpeed)


def get_params(candidate):
  fp = {0: dict(HYBRID_BUS0), 1: {}, 2: {}}
  return CarInterface.get_params(candidate, fp, [], False, False, False)


def get_params_sp(candidate):
  fp = {0: dict(HYBRID_BUS0), 1: {}, 2: {}}
  return CarInterface.get_params_sp(get_params(candidate), candidate, fp, [], False, False, False)


if __name__ == "__main__":
  unittest.main()
