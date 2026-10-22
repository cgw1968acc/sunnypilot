from types import SimpleNamespace

from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.gains import get_highway_friction_scale


def _ext(enabled=True):
  return SimpleNamespace(enforce_torque_control_toggle=False, frame=-1, params=None, hwy_friction=enabled,
                         _fric_base=None, _fric_applied=None, _v_ego=0.0)


def _params(friction=0.1):
  tp = structs.CarParams.LateralTorqueTuning().as_builder() if hasattr(structs.CarParams.LateralTorqueTuning(), 'as_builder') \
    else structs.CarParams.LateralTorqueTuning()
  tp.friction = friction
  return tp


def test_schedule():
  # tncr 2026-10-22: starts at 80 km/h (was 60); 1.4 at 90 (was 70); 100/110 unchanged
  assert get_highway_friction_scale(50 / 3.6) == 1.0
  assert get_highway_friction_scale(70 / 3.6) == 1.0
  assert get_highway_friction_scale(80 / 3.6) == 1.0
  assert abs(get_highway_friction_scale(85 / 3.6) - 1.2) < 1e-9
  assert get_highway_friction_scale(90 / 3.6) == 1.4
  assert abs(get_highway_friction_scale(95 / 3.6) - 1.45) < 1e-9
  assert get_highway_friction_scale(100 / 3.6) == 1.5
  assert abs(get_highway_friction_scale(105 / 3.6) - 1.55) < 1e-9
  assert get_highway_friction_scale(110 / 3.6) == 1.6
  assert get_highway_friction_scale(130 / 3.6) == 1.6


def test_no_compounding_and_follows_a_new_base():
  ext, tp = _ext(), _params(0.1)
  ext._v_ego = 100 / 3.6
  for _ in range(500):
    LatControlTorqueExt.update_override_torque_params(ext, tp)
  assert abs(tp.friction - 0.15) < 1e-6  # 1.5 at 100 km/h since 2026-10-13 (the test still said 1.4; fixed 10-16)
  tp.friction = 0.2  # torqued / the override sets a new value
  LatControlTorqueExt.update_override_torque_params(ext, tp)
  assert abs(tp.friction - 0.30) < 1e-6
  ext._v_ego = 40 / 3.6
  LatControlTorqueExt.update_override_torque_params(ext, tp)
  assert abs(tp.friction - 0.2) < 1e-6


def test_switch_off_is_stock():
  ext, tp = _ext(False), _params(0.1)
  ext._v_ego = 100 / 3.6
  for _ in range(10):
    LatControlTorqueExt.update_override_torque_params(ext, tp)
  assert abs(tp.friction - 0.1) < 1e-6
