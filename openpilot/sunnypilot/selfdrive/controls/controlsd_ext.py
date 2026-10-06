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
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0


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
