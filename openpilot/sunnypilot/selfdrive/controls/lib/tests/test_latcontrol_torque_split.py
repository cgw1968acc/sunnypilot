"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_split import LatControlTorqueSplit, get_v0_weight


class FakeExt:
  def __init__(self):
    self.calls = []

  def update_limits(self): self.calls.append('limits')
  def update_model_v2(self, m): self.calls.append(('model', m))
  def update_lateral_lag(self, lag): self.calls.append(('lag', lag))


class FakeLac:
  def __init__(self, torque):
    self.torque = torque
    self.extension = FakeExt()
    self.updates = 0
    self.params = None
    self.resets = 0
    self.pid = SimpleNamespace(i=0.0)

  def update_torque_parameters(self, *a): self.params = a
  def reset(self): self.resets += 1

  def update(self, active, CS, *a):
    self.updates += 1
    return self.torque, 0.0, SimpleNamespace(output=self.torque, name=id(self))


def make():
  CP = SimpleNamespace(steerLimitTimer=0.4)
  v1, v0 = FakeLac(0.2), FakeLac(0.1)
  return LatControlTorqueSplit(v1, v0, CP, None, None, 0.01), v1, v0


def step(lac, kph, active=True):
  return lac.update(active, SimpleNamespace(vEgo=kph / 3.6), None, None, False, 0.0, None, False, 0.2)


def test_weight_schedule():
  assert get_v0_weight(40 / 3.6) == 0.0
  assert get_v0_weight(55 / 3.6) == 0.0
  assert abs(get_v0_weight(60 / 3.6) - 0.5) < 1e-9
  assert get_v0_weight(65 / 3.6) == 1.0
  assert get_v0_weight(110 / 3.6) == 1.0


def test_v1_below_v0_above_blend_between():
  lac, v1, v0 = make()
  assert step(lac, 40)[0] == 0.2
  assert step(lac, 100)[0] == 0.1
  assert abs(step(lac, 60)[0] - 0.15) < 1e-9


def test_both_run_every_frame_so_the_next_owner_is_warm():
  lac, v1, v0 = make()
  for kph in (30, 60, 100):
    step(lac, kph)
  assert v1.updates == 3 and v0.updates == 3


def test_torque_is_continuous_through_the_band():
  lac, _, _ = make()
  prev = None
  for i in range(401):
    t = step(lac, 50 + i * 0.05)[0]  # 50 -> 70 km/h
    if prev is not None:
      assert abs(t - prev) < 0.001
    prev = t


def test_log_follows_the_owner_and_carries_the_blended_output():
  lac, v1, v0 = make()
  _, _, log = step(lac, 58)
  assert log.name == id(v1) and abs(log.output - (0.7 * 0.2 + 0.3 * 0.1)) < 1e-9
  _, _, log = step(lac, 62)
  assert log.name == id(v0)


def test_fan_out_reaches_both():
  lac, v1, v0 = make()
  lac.extension.update_limits()
  lac.extension.update_model_v2('m')
  lac.extension.update_lateral_lag(0.3)
  lac.update_torque_parameters(2.5, 0.0, 0.1)
  lac.reset()
  for c in (v1, v0):
    assert c.extension.calls == ['limits', ('model', 'm'), ('lag', 0.3)]
    assert c.params == (2.5, 0.0, 0.1) and c.resets == 1


def test_integrator_is_shared_so_the_idle_controller_does_not_wind_up():
  lac, v1, v0 = make()
  v1.pid.i, v0.pid.i = 0.05, 0.9  # v0 idle at low speed and wound up on its own
  step(lac, 40)
  assert v0.pid.i == v1.pid.i == 0.05
  v1.pid.i, v0.pid.i = 0.7, -0.1  # v1 idle on the highway
  step(lac, 100)
  assert v1.pid.i == v0.pid.i == -0.1
  v1.pid.i, v0.pid.i = 0.2, 0.4
  step(lac, 60)
  assert abs(v1.pid.i - 0.3) < 1e-9 and v0.pid.i == v1.pid.i


def test_v0_gets_v1_lane_centering():
  lac, v1, v0 = make()
  v0.centering_curvature = 0.0
  pc = SimpleNamespace(lane_centering=SimpleNamespace(correction=2e-4), low_centering=SimpleNamespace(correction=-5e-5))
  v1.extension.path_corrections = pc
  step(lac, 100)
  assert abs(v0.centering_curvature - 1.5e-4) < 1e-12
  pc.low_centering = None
  step(lac, 100)
  assert abs(v0.centering_curvature - 2e-4) < 1e-12


def test_v0_centering_without_path_corrections_is_zero():
  lac, v1, v0 = make()
  step(lac, 100)
  assert v0.centering_curvature == 0.0
