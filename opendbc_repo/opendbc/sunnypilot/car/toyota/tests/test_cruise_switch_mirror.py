import os
import tempfile
import unittest

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.toyota import cruise_switch_mirror as csm
from opendbc.sunnypilot.car.toyota.values import ToyotaSafetyFlagsSP

GENUINE_IDLE = bytes.fromhex("8640000056434281")      # findings6 payload shape, MAIN on + engaged, no press
GENUINE_PRESSED = bytes.fromhex("a640000056434281")   # same frame with RES/+ held


def make_cp_sp(pcm_cruise_speed=False, flag=True):
  cp_sp = structs.CarParamsSP()
  cp_sp.pcmCruiseSpeed = pcm_cruise_speed
  cp_sp.safetyParam = ToyotaSafetyFlagsSP.CRUISE_SWITCH_TX if flag else 0
  return cp_sp


class FakeCarState:
  def __init__(self):
    self.out = structs.CarState()
    self.cruise_switch = csm.CruiseSwitchRaw()


class Harness:
  """Runs the 100 Hz controller loop with a genuine 0x361 frame every `period` ticks."""

  def __init__(self, ctrl, period=6, genuine=GENUINE_IDLE):
    self.ctrl = ctrl
    self.CS = FakeCarState()
    self.frame = 0
    self.period = period
    self.genuine = genuine
    self.sent = []  # (frame, CanData)

  def rx(self, dat=None, src=0):
    # card delivers plain (address, dat, src) tuples, not CanData; exercise that shape in the harness
    self.CS.cruise_switch.update([(0, [(0x361, dat or self.genuine, src)])])

  def run(self, ticks, engaged=True, genuine_from=None):
    for _ in range(ticks):
      if self.frame % self.period == 0:
        self.rx(genuine_from(self.frame) if genuine_from else None)
      for msg in self.ctrl.update(self.CS, self.frame, engaged):
        self.sent.append((self.frame, msg))
      self.frame += 1


class TestCruiseSwitchMirror(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.mkdtemp()
    self.ctrl = csm.CruiseSwitchMirrorCarController(structs.CarParams(), make_cp_sp())
    self.ctrl.trigger_file = os.path.join(self.tmp, "trigger")
    self.ctrl.result_file = os.path.join(self.tmp, "result")

  def trigger(self, text=""):
    with open(self.ctrl.trigger_file, "w") as f:
      f.write(text)

  def result(self):
    with open(self.ctrl.result_file) as f:
      return f.read()

  def test_disabled_without_flag_or_with_pcm_speed(self):
    for cp_sp in (make_cp_sp(flag=False), make_cp_sp(pcm_cruise_speed=True)):
      ctrl = csm.CruiseSwitchMirrorCarController(structs.CarParams(), cp_sp)
      self.assertFalse(ctrl.enabled)
      ctrl.trigger_file = self.ctrl.trigger_file
      self.trigger()
      h = Harness(ctrl)
      h.run(300)
      self.assertEqual(h.sent, [])
      self.assertTrue(os.path.exists(ctrl.trigger_file))  # never consumed
      os.remove(ctrl.trigger_file)

  def test_idle_sends_nothing_and_leaves_no_result(self):
    h = Harness(self.ctrl)
    h.run(500)
    self.assertEqual(h.sent, [])
    self.assertFalse(os.path.exists(self.ctrl.result_file))

  def test_res_burst_mirrors_template_mid_gap(self):
    h = Harness(self.ctrl, period=6)
    h.run(60)  # fill the stability window
    self.trigger("res")
    h.run(200)

    # nothing may go out before the first genuine frame that follows arming (poll at frame 100, genuine at 102)
    self.assertEqual(h.sent[0][0], 102 + csm.SEND_DELAY_FRAMES)

    self.assertEqual(len(h.sent), csm.BURST_FRAMES)
    self.assertFalse(os.path.exists(self.ctrl.trigger_file))
    for frame, msg in h.sent:
      self.assertEqual(msg.address, 0x361)
      self.assertEqual(msg.src, 0)
      self.assertEqual(msg.dat, GENUINE_PRESSED)              # only bit 5 added, bytes 1-7 verbatim
      self.assertEqual(frame % h.period, csm.SEND_DELAY_FRAMES)  # 30 ms after each genuine frame
    # one mirrored frame per genuine frame, so the burst spans BURST_FRAMES genuine periods
    self.assertEqual(h.sent[-1][0] - h.sent[0][0], (csm.BURST_FRAMES - 1) * h.period)
    self.assertIn("done: 8 res frames sent", self.result())
    self.assertFalse(self.ctrl.armed)

  def test_set_burst_uses_bit_4(self):
    h = Harness(self.ctrl)
    h.run(60)
    self.trigger("set\n")
    h.run(200)
    self.assertEqual(len(h.sent), csm.BURST_FRAMES)
    self.assertTrue(all(msg.dat[0] == GENUINE_IDLE[0] | csm.BTN_SET for _, msg in h.sent))

  def test_long_press_variant_sends_24_frames(self):
    h = Harness(self.ctrl)
    h.run(60)
    self.trigger("res_long")
    h.run(400)
    self.assertEqual(len(h.sent), csm.LONG_PRESS_FRAMES)
    self.assertTrue(all(msg.dat == GENUINE_PRESSED for _, msg in h.sent))
    self.assertEqual(h.sent[-1][0] - h.sent[0][0], (csm.LONG_PRESS_FRAMES - 1) * h.period)  # ~1.5 s on the bus
    self.assertIn("done: 24 res_long frames sent", self.result())

  def test_unknown_button_is_ignored(self):
    h = Harness(self.ctrl)
    h.run(60)
    self.trigger("cancel")
    h.run(200)
    self.assertEqual(h.sent, [])
    self.assertIn("unknown button", self.result())

  def test_abort_when_not_engaged(self):
    h = Harness(self.ctrl)
    h.run(60)
    self.trigger("res")
    h.run(50)  # trigger is polled at frame 100, first genuine frame after arming is 102, first send 105
    self.assertGreater(len(h.sent), 0)
    n = len(h.sent)
    h.run(100, engaged=False)
    self.assertEqual(len(h.sent), n)
    self.assertIn("abort: cruise not engaged", self.result())

  def test_abort_when_driver_is_pressing(self):
    h = Harness(self.ctrl, genuine=GENUINE_PRESSED)
    h.run(60)
    self.trigger("res")
    h.run(100)
    self.assertEqual(h.sent, [])
    self.assertIn("driver is pressing", self.result())

  def test_abort_when_byte0_unstable(self):
    h = Harness(self.ctrl)
    h.run(60)
    self.trigger("res")
    # MAIN/ENGAGED bits toggling between genuine frames trips the stability gate
    h.run(100, genuine_from=lambda f: GENUINE_IDLE if (f // 6) % 2 else bytes([0x84]) + GENUINE_IDLE[1:])
    self.assertEqual(h.sent, [])
    self.assertIn("not stable", self.result())

  def test_abort_on_timeout_without_genuine_frames(self):
    h = Harness(self.ctrl)
    h.run(60)
    self.trigger("res")
    h.period = 10**9  # genuine stream stops
    h.run(csm.BURST_TIMEOUT_FRAMES + 150)
    self.assertEqual(h.sent, [])
    self.assertIn("abort: timeout", self.result())

  def test_own_echo_on_bus_128_is_not_a_template(self):
    raw = csm.CruiseSwitchRaw()
    raw.update([(0, [CanData(0x361, GENUINE_PRESSED, 128), CanData(0x361, GENUINE_IDLE, 2), CanData(0x360, GENUINE_IDLE, 0)])])
    self.assertEqual(raw.seq, 0)
    raw.update([(0, [CanData(0x361, GENUINE_IDLE, 0)])])
    self.assertEqual(raw.seq, 1)
    self.assertEqual(raw.latest, GENUINE_IDLE)

  def test_accepts_plain_tuples_and_can_data(self):
    raw = csm.CruiseSwitchRaw()
    raw.update([(0, [(0x361, GENUINE_IDLE, 0)]), (1, [CanData(0x361, GENUINE_IDLE, 0)])])
    self.assertEqual(raw.seq, 2)
    # bytearray payloads (capnp readers) are normalized to bytes
    raw.update([(2, [(0x361, bytearray(GENUINE_IDLE), 0)])])
    self.assertEqual(raw.latest, GENUINE_IDLE)
    self.assertIsInstance(raw.latest, bytes)


if __name__ == "__main__":
  unittest.main()
