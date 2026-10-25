"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom

from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.lead_brake_boost import LeadBrakeBoost
from openpilot.sunnypilot.selfdrive.controls.lib.mid_speed_brake_add import MidSpeedBrakeAdd
from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap_hold import StopGapHold
from openpilot.sunnypilot.selfdrive.controls.lib.creep_stall_guard import CreepStallGuard
from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap_close import StopGapClose
from openpilot.sunnypilot.selfdrive.controls.lib.end_force_limit import EndForceLimit
from openpilot.sunnypilot.selfdrive.controls.lib.stopping_controller import StoppingController
from openpilot.sunnypilot.selfdrive.controls.lib.cruise_smoother import CruiseSmoother
from openpilot.sunnypilot.selfdrive.controls.lib.highway_plan_curvature import HighwayPlanCurvature
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_split import LatControlTorqueSplit, SPLIT_ENABLED


# cap how fast the commanded curvature builds at the start of a lane change so the switch eases in instead of
# snapping over (ported from tncr18 / Corolla Cross feedback 2026-09-23). The cap ramps with time-in-lane-change:
# gentle for the first second, then opening up so the move still completes. 1/m per second, interpolated over seconds
# since the start. The ramp is also speed dependent (feedback 2026-09-24: at 70-80 km/h the gentle ramp left too
# little steering to finish the change): at or below 50 km/h the original flat 0.004 cap applies, at or above 100 km/h
# the 0.001->0.003 ramp applies, and each ramp point is interpolated linearly with speed in between.
LANE_CHANGE_START_CURV_RATE_T = (1.0, 3.0)                    # seconds since laneChangeStarting began
LANE_CHANGE_START_CURV_RATE_BP = (50. * CV.KPH_TO_MS, 100. * CV.KPH_TO_MS)  # m/s
LANE_CHANGE_START_CURV_RATE_V1 = (0.004, 0.001)              # 1/m per second at t <= 1 s, low / high speed
LANE_CHANGE_START_CURV_RATE_V3 = (0.004, 0.003)              # 1/m per second at t >= 3 s, low / high speed


def _lane_change_start_curv_rate(t: float, v_ego: float) -> float:
  t1, t3 = LANE_CHANGE_START_CURV_RATE_T
  r1 = float(np.interp(v_ego, LANE_CHANGE_START_CURV_RATE_BP, LANE_CHANGE_START_CURV_RATE_V1))
  r3 = float(np.interp(v_ego, LANE_CHANGE_START_CURV_RATE_BP, LANE_CHANGE_START_CURV_RATE_V3))
  if t <= t1:
    return r1
  if t >= t3:
    return r3
  return r1 + (r3 - r1) * (t - t1) / (t3 - t1)


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    self.lane_change_start_t = 0.0
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()
    self.lead_brake_boost = LeadBrakeBoost()  # tnpb2: 3-30 km/h x1.2 after a hard lead brake, see lead_brake_boost.py
    self.mid_speed_brake_add = MidSpeedBrakeAdd()  # tncr: +0.2 m/s^2 braking 30-10 km/h, see mid_speed_brake_add.py
    self.stop_gap_hold = StopGapHold()  # tnpb3: <30 km/h no lighter while the stop gap < 3.5 m, see stop_gap_hold.py
    self.creep_stall_guard = CreepStallGuard()  # tnpb3: 1-5 km/h adds back brake the hybrid creep eats
    self.stop_gap_close = StopGapClose()  # tnpb3: <12 km/h a stop predicted beyond 4 m crawls on at 2.3 km/h to 3.25 m
    self.end_force_limit = EndForceLimit()  # tnpb3: owner rule, <2 km/h linear to -0.30 at 0 km/h, never firmer
    self.stopping_controller = StoppingController(CP.stopAccel)  # end of stop / standstill hold, see stopping_controller.py
    self.cruise_smoother = CruiseSmoother()  # steady-cruise ripple, see cruise_smoother.py
    self.highway_plan_curvature = HighwayPlanCurvature()  # highway curvature swing, see highway_plan_curvature.py
    # LongControl's state and last output as they stood before this frame's update; long_target_sp() records them,
    # long_accel_sp() hands them to the stopping controller
    self._stopping_prev = None
    self._a_target_sp = 0.0

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP']
    self.pm_services_ext = ['carControlSP']

  def initialize_lateral_control(self, lac, CI, dt):
    enforce_torque_control = self.params.get_bool("EnforceTorqueControl")
    torque_versions = self.params.get("TorqueControlTune")
    if not enforce_torque_control:
      if self.CP.lateralTuning.which() == 'torque':
        return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)  # FIXME-SP: revert when upstream fixes tuning issues with v1
      return lac

    if torque_versions == 0.0:  # v0
      return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)
    elif SPLIT_ENABLED and self.CP.lateralTuning.which() == 'torque':
      # tnpb2: v1 below 55 km/h, v0 above 65 km/h, cross-faded in between (latcontrol_torque_split.py)
      return LatControlTorqueSplit(lac, LatControlTorqueV0(self.CP, self.CP_SP, CI, dt), self.CP, self.CP_SP, CI, dt)
    else:
      return lac

  def get_params_sp(self, sm: messaging.SubMaster) -> None:
    if time.monotonic() - self._param_update_time > PARAMS_UPDATE_PERIOD:
      self.blinker_pause_lateral.get_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, sm["lateralDelay"].lateralDelay)

      self._param_update_time = time.monotonic()

  def get_lat_active(self, sm: messaging.SubMaster) -> bool:
    if self.blinker_pause_lateral.update(sm['carState']):
      return False

    ss_sp = sm['selfdriveStateSP']
    if ss_sp.mads.available:
      return bool(ss_sp.mads.active)

    # MADS not available, use stock state to engage
    return bool(sm['selfdriveState'].active)

  @staticmethod
  def get_lead_data(_lead, src: log.RadarState.LeadData) -> None:
    _lead.dRel = src.dRel
    _lead.yRel = src.yRel
    _lead.vRel = src.vRel
    _lead.aRel = src.deprecated.aRel
    _lead.vLead = src.vLead
    _lead.dPath = src.deprecated.dPath
    _lead.vLat = src.deprecated.vLat
    _lead.vLeadK = src.vLeadK
    _lead.aLeadK = src.aLeadK
    _lead.fcw = src.deprecated.fcw
    _lead.status = src.present
    _lead.aLeadTau = src.aLeadTau
    _lead.modelProb = src.modelProb
    _lead.radar = src.radar
    _lead.radarTrackId = src.radarTrackId

  def state_control_ext(self, sm: messaging.SubMaster) -> custom.CarControlSP:
    CC_SP = custom.CarControlSP.new_message()

    self.get_lead_data(CC_SP.leadOne, sm['radarState'].leadOne)
    self.get_lead_data(CC_SP.leadTwo, sm['radarState'].leadTwo)

    # MADS state
    mads_src = sm['selfdriveStateSP'].mads
    CC_SP.mads.state = mads_src.state
    CC_SP.mads.enabled = mads_src.enabled
    CC_SP.mads.active = mads_src.active
    CC_SP.mads.available = mads_src.available

    # ICBM state
    icbm_src = sm['selfdriveStateSP'].intelligentCruiseButtonManagement
    CC_SP.intelligentCruiseButtonManagement.state = icbm_src.state
    CC_SP.intelligentCruiseButtonManagement.sendButton = icbm_src.sendButton
    CC_SP.intelligentCruiseButtonManagement.vTarget = icbm_src.vTarget

    return CC_SP

  @staticmethod
  def publish_ext(CC_SP: custom.CarControlSP, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    cc_sp_send = messaging.new_message('carControlSP')
    cc_sp_send.valid = sm['carState'].canValid
    cc_sp_send.carControlSP = CC_SP

    pm.send('carControlSP', cc_sp_send)

  def run_ext(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    CC_SP = self.state_control_ext(sm)
    self.publish_ext(CC_SP, sm, pm)

  def lane_change_start_rate_sp(self, new_desired_curvature: float, desired_curvature: float, lane_change_state,
                                v_ego: float) -> float:
    """tnpb2: called by stock controlsd after it takes the model's new desired curvature; limits how fast it moves
    away from the previous one during laneChangeStarting."""
    if lane_change_state == log.LaneChangeState.laneChangeStarting:
      self.lane_change_start_t += DT_CTRL
      max_lc_delta = _lane_change_start_curv_rate(self.lane_change_start_t, v_ego) * DT_CTRL
      return min(max(new_desired_curvature, desired_curvature - max_lc_delta), desired_curvature + max_lc_delta)
    self.lane_change_start_t = 0.0
    return new_desired_curvature

  def long_target_sp(self, long_plan, CS, long_active: bool) -> float:
    """sunnypilot hook before the stock LongControl.update(): returns the aTarget it should track (steady-cruise
    smoothing) and records LongControl's state from before the update for long_accel_sp()."""
    self._stopping_prev = (self.LoC.long_control_state, self.LoC.last_output_accel)
    self._a_target_sp = self.cruise_smoother.update(long_plan.aTarget, CS.vEgo, self.sm['radarState'].leadOne.present,
                                                    long_active, long_plan.shouldStop)
    return self._a_target_sp

  def long_accel_sp(self, actuators, CS, long_plan, accel_limits: tuple[float, float], long_active: bool) -> None:
    """sunnypilot hook after the stock LongControl.update(): the end-of-stop policy (stopping controller), then the
    send-only shaping layers, the owner's end-force ceiling last."""
    prev_state, prev_accel = self._stopping_prev
    stock_state = self.LoC.long_control_state
    lead_one = self.sm['radarState'].leadOne
    self.LoC.long_control_state, accel = self.stopping_controller.update(
      prev_state, stock_state, CS, self._a_target_sp, prev_accel, self.LoC.last_output_accel, accel_limits, long_plan.hasLead,
      pitch=self.calibrated_pose.orientation.pitch if self.calibrated_pose is not None else None,
      a_long=self.calibrated_pose.acceleration.x if self.calibrated_pose is not None else None,
      lead_d=lead_one.dRel if lead_one.present else None,
      lead_v=lead_one.vLead if lead_one.present else None)
    if self.LoC.long_control_state != stock_state:
      self.LoC.reset()
    self.LoC.last_output_accel = accel
    # the stop-gap hold, creep-stall guard and boost only shape what is sent; last_output_accel (the next frame's
    # prev_accel for the stopping controller and LongControl's stopping state) keeps the plain request, or the boost
    # would compound
    accel_shaped = self.stop_gap_hold.update(accel, CS.vEgo, CS.aEgo, CS.standstill, lead_one, long_active)
    accel_shaped = self.stop_gap_close.update(accel_shaped, CS.vEgo, CS.aEgo, CS.standstill, lead_one, long_active)
    # the gap-close crawl rolls on purpose: not a creep stall
    accel_shaped = self.creep_stall_guard.update(accel_shaped, CS.vEgo, CS.aEgo, CS.standstill,
                                                 long_active and not self.stop_gap_close.active, accel_limits[0])
    accel_shaped = self.lead_brake_boost.update(accel_shaped, CS.vEgo, CS.standstill, lead_one, long_active,
                                                accel_limits[0], a_ego=CS.aEgo,
                                                hold_off=self.stop_gap_hold.roll_out is not None)
    # tncr: +0.2 m/s^2 on braking 30-10 km/h (gone by 6 km/h, before the end ceiling below)
    accel_shaped = self.mid_speed_brake_add.update(accel_shaped, CS.vEgo, CS.standstill, long_active, accel_limits[0],
                                                   hold_off=self.stop_gap_hold.roll_out is not None or
                                                   self.stop_gap_close.active)
    # owner's end-force rule, last: never firmer than the end ceiling (Cross: -2.0 at 5 km/h -> -0.34 at 0, and the
    # anchored line below 2 km/h), whatever asked for more - the roll guard included (2026-10-15 evening: its exemption
    # put 4120 N on at 2.7 km/h and the car stopped with a jolt); the standstill hold then firms up at HOLD_RATE
    # downhill: the owner's approved exception, a little firmer with the grade below 30 km/h (end_force_limit.py)
    actuators.accel = float(self.end_force_limit.update(accel_shaped, CS.vEgo, CS.standstill, long_active, CS.aEgo,
                                                        pitch=self.calibrated_pose.orientation.pitch
                                                        if self.calibrated_pose is not None else None))

  def desired_curvature_sp(self, new_desired_curvature: float, model_v2, v_ego: float, model_lat_active: bool) -> float:
    """sunnypilot hook after stock controlsd picks the new desired curvature: lane-change start rate, then the
    highway plan-curvature smoothing (model path only, not in a lane change)."""
    lane_change_state = model_v2.meta.laneChangeState
    new_desired_curvature = self.lane_change_start_rate_sp(new_desired_curvature, self.desired_curvature,
                                                           lane_change_state, v_ego)
    return self.highway_plan_curvature.update(new_desired_curvature, model_v2, v_ego, model_lat_active,
                                              lane_change_state == log.LaneChangeState.off)
