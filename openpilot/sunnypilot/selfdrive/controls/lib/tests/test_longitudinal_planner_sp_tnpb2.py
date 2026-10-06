import math
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.lead_start_assist.lead_start_assist import LeadStartAssist
from openpilot.sunnypilot.selfdrive.controls.lib.set_speed_ramp import SetSpeedRamp
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP, SET_SPEED_INTENT_T

DT = 0.05


def TnPlannerExt(dt):
  """only the tnpb2 part of LongitudinalPlannerSP (its full __init__ needs the car and the MPC)"""
  # the tnpb2 methods of LongitudinalPlannerSP need only these attributes; its full __init__ needs the car and the MPC
  sp = LongitudinalPlannerSP.__new__(LongitudinalPlannerSP)
  sp.planner_ext_enabled = True
  sp._sp_dt = dt
  sp.lead_start_assist = LeadStartAssist(dt)
  sp._sp_v_cruise_prev = 0.0
  sp._sp_t_set_speed_up = SET_SPEED_INTENT_T
  sp.set_speed_ramp = SetSpeedRamp()
  sp._sp_set_speed_up = False
  return sp


def sm_with_pitch(pitch_deg):
  return {'carControl': SimpleNamespace(orientationNED=[0.0, math.radians(pitch_deg), 0.0])}


def test_set_speed_increase_overrides_the_throttle_gate_for_a_while():
  ext = TnPlannerExt(DT)
  assert ext.update_allow_throttle_sp(False, 10.0, 12.0, True, False) is False      # first frame only learns the set speed
  assert ext.update_allow_throttle_sp(False, 10.0, 13.5, True, False) is True       # + pressed
  n = 1
  while ext.update_allow_throttle_sp(False, 10.0, 13.5, True, False):
    n += 1
  assert abs(n * DT - SET_SPEED_INTENT_T) <= 2 * DT


def test_gate_open_stays_open():
  assert TnPlannerExt(DT).update_allow_throttle_sp(True, 10.0, 12.0, True, False) is True


def test_uphill_scales_the_ceiling_downhill_untouched():
  ext = TnPlannerExt(DT)
  assert abs(ext.max_accel_sp(1.0, sm_with_pitch(5.0)) - 0.8 * (1.0 - 0.25 * 0.0) * 1.0) < 0.06
  assert ext.max_accel_sp(1.0, sm_with_pitch(-6.0)) == 1.0
  assert ext.max_accel_sp(None, sm_with_pitch(5.0)) is None
