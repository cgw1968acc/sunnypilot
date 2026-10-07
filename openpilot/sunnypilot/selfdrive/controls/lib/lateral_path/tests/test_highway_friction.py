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
  assert get_highway_friction_scale(50 / 3.6) == 1.0
  assert abs(get_highway_friction_scale(65 / 3.6) - 1.15) < 1e-9
  assert get_highway_friction_scale(100 / 3.6) == 1.3


def test_no_compounding_and_follows_a_new_base():
  ext, tp = _ext(), _params(0.1)
  ext._v_ego = 100 / 3.6
  for _ in range(500):
    LatControlTorqueExt.update_override_torque_params(ext, tp)
  assert abs(tp.friction - 0.13) < 1e-6
  tp.friction = 0.2  # torqued / the override sets a new value
  LatControlTorqueExt.update_override_torque_params(ext, tp)
  assert abs(tp.friction - 0.26) < 1e-6
  ext._v_ego = 40 / 3.6
  LatControlTorqueExt.update_override_torque_params(ext, tp)
  assert abs(tp.friction - 0.2) < 1e-6


def test_switch_off_is_stock():
  ext, tp = _ext(False), _params(0.1)
  ext._v_ego = 100 / 3.6
  for _ in range(10):
    LatControlTorqueExt.update_override_torque_params(ext, tp)
  assert abs(tp.friction - 0.1) < 1e-6
