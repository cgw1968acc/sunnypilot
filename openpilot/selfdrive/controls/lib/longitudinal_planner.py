#!/usr/bin/env python3
import math
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.lead_start_assist.lead_start_assist import LeadStartAssist
from openpilot.sunnypilot.selfdrive.controls.lib.set_speed_ramp import SetSpeedRamp


A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MIN = -1.2
# Set-speed reductions at highway speed: ease off the throttle very gradually (Altis driver 2026-09-29: consecutive
# -5 km/h taps above 90 km/h felt like the throttle being dropped; rlog 2026-09-27 route 000000c3, 112 -> 107 km/h at
# 105: the cruise target went +0.12 -> -0.37 m/s^2 in 0.5 s). While v_ego is above V_CRUISE_EASE_MIN and the cruise
# target is below v_ego, the cruise candidate's DOWNWARD jerk is limited to J_CRUISE_EASE_DOWN (+0.12 -> -0.35 now
# takes ~2.4 s). The lead MPC and e2e candidates are separate, so braking for a lead is unaffected, and the upward
# (release) side keeps J_CRUISE.
V_CRUISE_EASE_MIN = 90. * CV.KPH_TO_MS  # m/s
J_CRUISE_EASE_DOWN = 0.2  # m/s^3
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
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
UPHILL_PITCH_FLAT_DEG = 1.0
UPHILL_ACCEL_BP = [1.0, 4.0, 8.0]  # deg of climb above flat
UPHILL_ACCEL_V = [1.0, 0.8, 0.6]  # factor on the max acceleration


def get_uphill_accel_factor(pitch_rad: float) -> float:
  climb = np.degrees(pitch_rad) - UPHILL_PITCH_FLAT_DEG
  return float(np.interp(climb, UPHILL_ACCEL_BP, UPHILL_ACCEL_V))

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle,
                      max_accel_override=None):
  if max_accel_override is not None:
    max_accel = max_accel_override
  else:
    max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)
  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  ease = not e2e and v_ego > V_CRUISE_EASE_MIN and v_cruise < v_ego
  j_down = min(j_cruise, J_CRUISE_EASE_DOWN) if ease else j_cruise
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_down * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True
    self.lead_start_assist = LeadStartAssist(self.dt)

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.v_cruise_prev = 0.0
    self.t_set_speed_up = SET_SPEED_INTENT_T  # s since the last set-speed increase
    self.set_speed_ramp = SetSpeedRamp()
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    force_decel = sm['controlsState'].forceDecel
    if force_decel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    set_speed_up = v_cruise_initialized and self.v_cruise_prev > 0.0 and v_cruise > self.v_cruise_prev + 0.1 and not force_decel
    if v_cruise_initialized and self.v_cruise_prev > 0.0 and v_cruise < self.v_cruise_prev - 0.1:
      self.set_speed_ramp.cancel()
    if set_speed_up:
      self.t_set_speed_up = 0.0
    else:
      self.t_set_speed_up = min(self.t_set_speed_up + self.dt, SET_SPEED_INTENT_T)
    self.v_cruise_prev = v_cruise if v_cruise_initialized else 0.0
    set_speed_intent = self.t_set_speed_up < SET_SPEED_INTENT_T and v_ego < v_cruise
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED or set_speed_intent

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target
      self.lead_start_assist.reset()

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)
    self.update_dec(sm)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    is_e2e = self.is_e2e(sm)

    max_accel_override = self.get_max_accel_override(v_ego, sm['carStateSP'].engineOff, sm['radarState'].leadOne)
    if max_accel_override is not None and len(sm['carControl'].orientationNED) == 3:
      max_accel_override *= get_uphill_accel_factor(sm['carControl'].orientationNED[1])
    # accel_coast is ACCEL_MAX when the orientation is not valid; pass it only when it is a real (negative) coast value
    v_cruise = self.get_cruise_target_override(v_ego, v_cruise, force_decel, accel_coast if accel_coast < 0.0 else None)
    a_cruise_prev = self.a_cruise
    gated_cruise = get_cruise_accel(is_e2e, v_cruise, v_ego, a_cruise_prev, steer_angle_without_offset,
                                    self.CP, self.dt, accel_coast, self.allow_throttle, max_accel_override)
    ungated_cruise = get_cruise_accel(is_e2e, v_cruise, v_ego, a_cruise_prev, steer_angle_without_offset,
                                      self.CP, self.dt, accel_coast, True, max_accel_override)
    # SP: smooth parabolic acceleration after a set-speed increase (cruise candidate only)
    if reset_state or force_decel:
      self.set_speed_ramp.cancel()
    elif set_speed_up:
      a_max = max_accel_override if max_accel_override is not None else get_max_accel(v_ego)
      self.set_speed_ramp.start(v_ego, v_cruise, a_cruise_prev, a_max)
    a_ramp = self.set_speed_ramp.update(self.dt, v_ego, v_cruise)
    if a_ramp is not None:
      gated_cruise = min(gated_cruise, a_ramp)
      ungated_cruise = min(ungated_cruise, a_ramp)
    self.a_cruise = self.arbitrate_cruise_candidate(
      sm, gated_cruise, ungated_cruise, output_a_target_mpc, self.mpc.source,
      allow_throttle=self.allow_throttle, e2e=is_e2e, force_decel=force_decel,
    )
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    lead_one = sm['radarState'].leadOne
    long_allowed = (not long_control_off and not force_decel and not sm['carState'].brakePressed and
                    not sm['carState'].gasPressed)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    if is_e2e:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)

    # Lead start assist: when stopped behind a lead that starts to move, put a floor under the target right away
    # instead of waiting for the lead MPC to close the gap. Never while the driver or e2e is asking for a stop.
    assist_allowed = long_allowed and not (is_e2e and output_should_stop_e2e)
    a_start = self.lead_start_assist.update(assist_allowed, v_ego, lead_one.present, lead_one.dRel, lead_one.vLead, lead_one.vRel)
    if a_start is not None and a_start > output_a_target:
      output_a_target = a_start
      self.output_should_stop = False
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)
    self.accel_controller_active = self.is_accel_controller_active(force_decel)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
