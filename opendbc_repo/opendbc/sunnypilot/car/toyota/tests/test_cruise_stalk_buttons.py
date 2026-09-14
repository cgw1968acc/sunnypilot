import unittest

from opendbc.car import structs
from opendbc.car.toyota.carstate import CarState, CruiseButton

ButtonType = structs.CarState.ButtonEvent.Type


class FakeParser:
  def __init__(self, res: list[int], set_: list[int]):
    self.vl_all = {"CLUTCH": {"CRUISE_BTN_RES": res, "CRUISE_BTN_SET": set_}}


def make_cs():
  cp = structs.CarParams(); cp.carFingerprint = "TOYOTA_COROLLA_TSS2"; cp.brand = "toyota"
  cp_sp = structs.CarParamsSP(); cp_sp.pcmCruiseSpeed = False
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
    import inspect
    src = inspect.getsource(CarState.get_can_parsers)
    self.assertIn('if not CP_SP.pcmCruiseSpeed:', src)
    self.assertIn('("CLUTCH", 15)', src)


if __name__ == "__main__":
  unittest.main()
