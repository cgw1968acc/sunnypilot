"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import math

import numpy as np

from openpilot.cereal import messaging, custom, log
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.accel_controller.accel_controller import AccelController
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle
from openpilot.sunnypilot.selfdrive.controls.lib.lead_start_assist.lead_start_assist import LeadStartAssist
from openpilot.sunnypilot.selfdrive.controls.lib.set_speed_ramp import SetSpeedRamp

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
MpcPlanSource = log.LongitudinalPlan.LongitudinalPlanSource

E2E_BRAKE_HOLD_ACCEL = -0.2  # m/s^2

# Set-speed reductions at highway speed: ease off the throttle very gradually (Altis driver 2026-09-29: consecutive
# -5 km/h taps above 90 km/h felt like the throttle being dropped; rlog 2026-09-27 route 000000c3, 112 -> 107 km/h at
# 105: the cruise target went +0.12 -> -0.37 m/s^2 in 0.5 s). While v_ego is above V_CRUISE_EASE_MIN and the cruise
# target is below v_ego, the cruise candidate's DOWNWARD jerk is limited to J_CRUISE_EASE_DOWN (+0.12 -> -0.35 now
# takes ~2.4 s). The lead MPC and e2e candidates are separate, so braking for a lead is unaffected, and the upward
# (release) side keeps J_CRUISE.
V_CRUISE_EASE_MIN = 90. * CV.KPH_TO_MS  # m/s
J_CRUISE_EASE_DOWN = 0.2  # m/s^3

# SP (Altis, driver 2026-10-04): a set-speed INCREASE is an explicit order to accelerate, so the model's throttle gate
# (gasPressProbs below ALLOW_THROTTLE_THRESHOLD clamps the cruise target to the coast decel) is ignored for
# SET_SPEED_INTENT_T after it. Route 000000f6 13:32:53: in the traffic-light curve at 40 km/h the driver pressed + from
# 50 to 74, the gate closed (gasProb 0.03-0.12) and the planner held -0.47 m/s^2 for 5.8 s; "the car does not react to
# + in this curve, after the junction + works again". Everything else (lead MPC, e2e candidate, limits) still applies.
SET_SPEED_INTENT_T = 6.0  # s
# SP (Altis, owner 2026-10-05): "I never liked the engine pulling harder on a climb; a little LESS acceleration uphill
# feels natural and safe - scale it by the grade, the steeper the weaker". The car's pitch (carControl.orientationNED,
# + = nose up; this car reads ~+1.0 deg on the flat, see auto_brake_hold.FLAT_PITCH_DEG) scales the accel profile's
# ceiling: full up to 1 deg above flat, 0.8x at 4 deg, 0.6x at 8 deg and beyond. Downhill is untouched.
# Owner 2026-10-11: "on a climb I want the speed recovered slowly - lower the uphill scaling a little more" ->
# 0.7x at 4 deg, 0.5x at 8 deg (was 0.8 / 0.6); full up to 1 deg unchanged, so the flat and gentle grades feel the same.
UPHILL_PITCH_FLAT_DEG = 1.0
UPHILL_ACCEL_BP = [1.0, 4.0, 8.0]  # deg of climb above flat
UPHILL_ACCEL_V = [1.0, 0.7, 0.5]  # factor on the max acceleration


def get_uphill_accel_factor(pitch_rad: float) -> float:
  climb = np.degrees(pitch_rad) - UPHILL_PITCH_FLAT_DEG
  return float(np.interp(climb, UPHILL_ACCEL_BP, UPHILL_ACCEL_V))


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.accel_controller = AccelController()
    self.accel_controller_active = False
    self.events_sp = EventsSP()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.
    self.lead_one = None


    # tnpb2 additions, called from five hooks in stock LongitudinalPlanner.update()
    self.params = Params()
    self._sp_dt = mpc.dt
    self.lead_start_assist = LeadStartAssist(mpc.dt)
    self._sp_v_cruise_prev = 0.0
    self._sp_t_set_speed_up = SET_SPEED_INTENT_T  # s since the last set-speed increase
    self.set_speed_ramp = SetSpeedRamp()
    self._sp_set_speed_up = False

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not experimental_mode:
      return False

    if not self.dec.active() or self.dec.mode() == "blended":
      return True

    if self.mpc.source == MpcPlanSource.e2e and sm['modelV2'].action.desiredAcceleration < E2E_BRAKE_HOLD_ACCEL:
      return True

    return False

  def get_max_accel_override(self, v_ego: float, engine_off: bool = False, lead=None) -> float | None:
    if not self.accel_controller.is_enabled():
      return None

    # the eco lead pull-away boost needs the lead; the stock planner does not pass it, so update() keeps it
    return self.accel_controller.get_max_accel(v_ego, engine_off, lead if lead is not None else self.lead_one)

  def get_cruise_target_override(self, v_ego: float, v_target: float, force_decel: bool, accel_coast: float | None = None) -> float:
    if not self.accel_controller.is_enabled() or force_decel or self.source != LongitudinalPlanSource.cruise:
      return v_target

    return self.accel_controller.get_cruise_target(v_ego, v_target, accel_coast)

  def is_accel_controller_active(self, force_decel: bool) -> bool:
    return bool(self.accel_controller.is_enabled() and not force_decel and
                self.mpc.source == MpcPlanSource.cruise)

  def _has_valid_selected_lead(self, sm: messaging.SubMaster, source: MpcPlanSource) -> bool:
    radar_valid = sm.valid.get('radarState', False) and getattr(sm, 'alive', {}).get('radarState', False)
    return radar_valid and ((source == MpcPlanSource.lead0 and sm['radarState'].leadOne.present) or
                            (source == MpcPlanSource.lead1 and sm['radarState'].leadTwo.present))

  def arbitrate_cruise_candidate(self, sm: messaging.SubMaster, gated: float, ungated: float,
                                 mpc_accel: float, mpc_source: MpcPlanSource, *, allow_throttle: bool,
                                 e2e: bool, force_decel: bool) -> float:
    finite = all(math.isfinite(value) for value in (gated, ungated, mpc_accel))
    coast_gate_changed_source = gated < mpc_accel <= ungated
    if (finite and not allow_throttle and not e2e and not force_decel
        and self._has_valid_selected_lead(sm, mpc_source) and coast_gate_changed_source):
      return ungated

    return gated

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

  def update(self, sm: messaging.SubMaster) -> None:
    self.lead_one = sm['radarState'].leadOne
    self.accel_controller.update()
    self.events_sp.clear()
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def update_dec(self, sm: messaging.SubMaster) -> None:
    self.dec.update(sm)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()
    dec.decelIntent = float(self.dec.signals.decel_intent)
    dec.curveDetected = bool(self.dec.signals.curve_detected)
    dec.wantBlended = bool(self.dec.want_blended)
    dec.leadVeto = bool(self.dec.lead_veto)

    accel_controller = longitudinalPlanSP.accelController
    accel_controller.enabled = bool(self.accel_controller.is_enabled())
    accel_controller.active = bool(self.accel_controller_active)
    accel_controller.profile = int(self.accel_controller.profile)

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)

  def update_allow_throttle_sp(self, allow_throttle: bool, v_ego: float, v_cruise: float, v_cruise_initialized: bool,
                               force_decel: bool) -> bool:
    """set-speed increase detection; an increase overrides the model's throttle gate for SET_SPEED_INTENT_T"""
    self._sp_set_speed_up = (v_cruise_initialized and self._sp_v_cruise_prev > 0.0 and v_cruise > self._sp_v_cruise_prev + 0.1
                             and not force_decel)
    if v_cruise_initialized and self._sp_v_cruise_prev > 0.0 and v_cruise < self._sp_v_cruise_prev - 0.1:
      self.set_speed_ramp.cancel()
    if self._sp_set_speed_up:
      self._sp_t_set_speed_up = 0.0
    else:
      self._sp_t_set_speed_up = min(self._sp_t_set_speed_up + self._sp_dt, SET_SPEED_INTENT_T)
    self._sp_v_cruise_prev = v_cruise if v_cruise_initialized else 0.0
    set_speed_intent = self._sp_t_set_speed_up < SET_SPEED_INTENT_T and v_ego < v_cruise
    return allow_throttle or set_speed_intent

  def reset_sp(self) -> None:
    self.lead_start_assist.reset()

  def max_accel_sp(self, max_accel_override: float | None, sm: messaging.SubMaster) -> float | None:
    """uphill scaling of the accel profile ceiling"""
    if max_accel_override is not None and len(sm['carControl'].orientationNED) == 3:
      max_accel_override *= get_uphill_accel_factor(sm['carControl'].orientationNED[1])
    return max_accel_override

  def cruise_accel_sp(self, gated: float, ungated: float, e2e: bool, v_cruise: float, v_ego: float, a_cruise_prev: float,
                      max_accel_override: float | None, reset_state: bool, force_decel: bool) -> tuple[float, float]:
    """highway set-speed-reduction ease and the smooth set-speed-increase ramp, on both cruise candidates"""
    from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MAX_BP, J_CRUISE_VALS, get_max_accel
    if not e2e and v_ego > V_CRUISE_EASE_MIN and v_cruise < v_ego:
      # same as clipping the stock cruise target's DOWNWARD step at J_CRUISE_EASE_DOWN instead of J_CRUISE
      j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
      floor = a_cruise_prev - min(j_cruise, J_CRUISE_EASE_DOWN) * self._sp_dt
      gated, ungated = float(max(gated, floor)), float(max(ungated, floor))
    if reset_state or force_decel:
      self.set_speed_ramp.cancel()
    elif self._sp_set_speed_up:
      a_max = max_accel_override if max_accel_override is not None else get_max_accel(v_ego)
      self.set_speed_ramp.start(v_ego, v_cruise, a_cruise_prev, a_max)
    a_ramp = self.set_speed_ramp.update(self._sp_dt, v_ego, v_cruise)
    if a_ramp is not None:
      gated = min(gated, a_ramp)
      ungated = min(ungated, a_ramp)
    return gated, ungated

  def lead_start_sp(self, output_a_target: float, output_should_stop: bool, sm: messaging.SubMaster, v_ego: float,
                    long_control_off: bool, force_decel: bool, is_e2e: bool, output_should_stop_e2e: bool) -> tuple[float, bool]:
    """Lead start assist: when stopped behind a lead that starts to move, put a floor under the target right away
    instead of waiting for the lead MPC to close the gap. Never while the driver or e2e is asking for a stop."""
    lead_one = sm['radarState'].leadOne
    long_allowed = (not long_control_off and not force_decel and not sm['carState'].brakePressed and
                    not sm['carState'].gasPressed)
    assist_allowed = long_allowed and not (is_e2e and output_should_stop_e2e)
    a_start = self.lead_start_assist.update(assist_allowed, v_ego, lead_one.present, lead_one.dRel, lead_one.vLead, lead_one.vRel,
                                            lead_one.vLeadK)
    if a_start is not None and a_start > output_a_target:
      return a_start, False
    return output_a_target, output_should_stop
