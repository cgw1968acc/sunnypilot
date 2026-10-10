import numpy as np

from cereal import custom
from opendbc.car import structs
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality import AccelPersonalityController, MAX_ACCEL_BP, MAX_ACCEL_PROFILES

# the branch's own single profile (stock-file A_CRUISE_MAX_VALS) = eco
ECO_VALS, ECO_BP = [1.6, 0.75, 0.28, 0.08], [0., 10.0, 25., 40.]


def stock_max(v):
  return float(np.interp(v, ECO_BP, ECO_VALS))


class FakeSM:
  def __init__(self, personality=None):
    self.data = {'carStateSP': custom.CarStateSP.new_message(accelPersonality=personality) if personality else None}
    self.recv_frame = {'carStateSP': 5 if personality else 0}

  def __getitem__(self, key):
    return self.data[key]


class TestAccelPersonality:
  def test_eco_before_the_car_reports(self):
    c = AccelPersonalityController()
    c.update(FakeSM())
    for v in np.arange(0, 45, 0.5):
      assert c.max_accel(v, stock_max(v)) == stock_max(v)

  def test_each_mode(self):
    c = AccelPersonalityController()
    for name in ('eco', 'normal', 'sport'):
      c.update(FakeSM(name))
      assert c.personality == name
      for v in np.arange(0, 45, 0.5):
        got = c.max_accel(v, stock_max(v))
        want = stock_max(v) if name == 'eco' else float(np.interp(v, MAX_ACCEL_BP, MAX_ACCEL_PROFILES[name]))
        assert abs(got - want) < 1e-9

  def test_order_eco_le_normal_le_sport(self):
    c = AccelPersonalityController()
    for v in np.arange(0, 45, 0.25):
      out = []
      for name in ('eco', 'normal', 'sport'):
        c.update(FakeSM(name))
        out.append(c.max_accel(v, stock_max(v)))
      assert out[0] <= out[1] + 1e-9 <= out[2] + 2e-9, (v, out)

  def test_never_above_toyota_accel_max(self):
    for prof in MAX_ACCEL_PROFILES.values():
      assert max(prof) <= 2.0


class TestCarStateSPMessage:
  def test_struct_to_capnp_roundtrip(self):
    for mode in structs.CarStateSP.AccelerationPersonality:
      cs = structs.CarStateSP(accelPersonality=mode)
      msg = convert_to_capnp(cs)
      assert str(msg.accelPersonality) == mode.value

  def test_default_is_normal(self):
    assert str(convert_to_capnp(structs.CarStateSP()).accelPersonality) == 'normal'
