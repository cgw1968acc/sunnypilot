import numpy as np
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap import end_decel, DELIVERY_LEAD
import openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap as stop_gap_mod

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

STOPPING_DECEL_RATE = 0.3  # m/s^2/s while trying to stop
# Once the car is actually standing still the brake request keeps falling towards stopAccel at this faster rate, so
# the hold firms up in well under a second instead of ~3 s (Altis 2026-09-27: after a stop behind a lead the car rolled
# on towards it). The approach and the stop itself are untouched: this rate only applies with CS.standstill set.
STANDSTILL_HOLD_RATE = 0.5  # m/s^2/s (1.0 -> 0.5, 2026-10-01: 2 of 18 stops rolled ~8 cm forward while the hold was ramping
                            # at 1.0 - PCM brake blending at standstill; the gradual creep response covers any creep)
# CS.standstill (all wheel speeds reading zero) arrives while the Altis is still settling the last few centimetres, so
# a hold ramp that starts on that flag is felt as the brake biting before the car is at rest (driver report
# 2026-09-29, first stop on the -1.0 hold). The fast ramp therefore waits until standstill has been continuous for
# this long; until then the request keeps falling only at the gentle STOPPING_DECEL_RATE.
# With a lead the hold matters more (creep towards it), without one the driver wants a longer settle first
# (2026-09-29: "autohold still steps in too early on a stop with no car ahead", asked for 0.8 s).
STANDSTILL_HOLD_DELAY_LEAD = 0.6  # s (0.5 -> 0.8 -> 0.6 over the 2026-09-30 tests: 0.8 was a little late)
STANDSTILL_HOLD_DELAY_NO_LEAD = 0.9  # s (0.8 -> 1.1 -> 0.9)
# Before that the request is frozen at the value the glide ended on: the driver wants the stop itself exactly as it
# felt on the drives where the hold never ramped at all (stopAccel -0.02 days) and the extra force only once the
# car is at rest. Safety net: if the car has not reached standstill within STOPPING_FREEZE_MAX of entering the
# stopping state (request too light to finish the stop, e.g. a creep), the upstream STOPPING_DECEL_RATE ramp resumes.
STOPPING_FREEZE_MAX = 2.0  # s
# Leaving the stopping state while standing still needs the "go" condition to hold for this long. Route 000000d9 seg 12
# (2026-09-30 16:58): a one-sample radar flicker made the planner drop should_stop for a single frame, the state went to
# pid for that frame (request +1.17), the hold was released and the car lurched forward at a stopped lead. A real
# departure (lead start assist confirms 0.25 s of motion, resume press) easily outlasts this.
STOPPING_EXIT_DEBOUNCE = 0.2  # s
# Before the wheels stop, the request may keep EASING with the planner's target (never firming): the stopping state
# starts at 0.9 km/h (STOPPING_SPEED) and freezing there kept the request at the curve's 1.2 km/h value, so the tuned
# last-metre easing (A_1KPH -> A_0KPH) never reached the car (2026-09-30 22:54:30, route 000000e0 seg 15: request
# frozen at -0.40 while the plan fell -0.37 -> -0.16; IMU decel flat at 0.75 to the stop, rebound +0.52 - the nod).
# The good stops (22:13:45 openpilot, the driver's own seamless ones) all taper in the last 0.3 s. A target above
# STOPPING_FOLLOW_MIN is not followed (a released brake at 0.5 km/h would lurch), and the easing is rate-limited.
# Creep response (driver 2026-09-30 night: the goal stays smoothness, so if the lighter end lets the car creep after
# it has stopped, add brake only a little, and more only if it keeps creeping). After the wheels have stopped once in
# this stopping episode, while the car is moving again and before the standstill hold has taken over, the request is
# deepened at CREEP_RATE_BASE plus CREEP_RATE_GROWTH per second of continued creep; as soon as the wheels stop again
# the request is held where it is and the normal delayed hold continues.
CREEP_V_MIN = 0.03  # m/s, below this the car counts as stopped for the creep response
CREEP_RATE_BASE = 0.3  # m/s^2/s
CREEP_RATE_GROWTH = 1.5  # m/s^2/s per second of creep
CREEP_RATE_MAX = 1.0  # m/s^2/s
STOPPING_FOLLOW_MIN = -0.10  # m/s^2
# While still rolling in the stopping state the request is at least this firm: below 2.5 km/h the hybrid's creep torque
# eats ~0.2 of a request (27 stops), so a lighter request glides instead of stopping (route 000000e4 11:37:20: -0.35 at
# 1.6 km/h held the speed for 0.6 s and the car braked a second time). 0.42 delivers ~0.22-0.3, the lightest perfect
# manual stop's final level.
STOPPING_ROLLING_MIN = -0.42  # m/s^2
STOPPING_FOLLOW_RATE = 2.0  # m/s^3
# Universal end-of-stop taper (driver 2026-09-30 night: the 'taper' in the last 0.3 s is what makes a stop seamless, a
# human cannot repeat it every time, the code must). Whatever produced the request - the stop-gap governor, the lead
# MPC or an e2e stop with no lead - below END_TAPER_V_START the braking request may not exceed the tuned end-of-stop
# curve (the governor's A_3KPH..A_0KPH), read END_TAPER_LEAD ahead of the car's speed so the deceleration the PCM
# actually delivers (lagged) follows it. Positive targets pass, so a lead driving off is not held back.
END_TAPER_V_START = 10.0 / 3.6  # m/s (3 -> 10 km/h, 2026-10-01 device campaign: with a lead braking -2 to a stop the MPC still
                                # asked 1.5 at 1 km/h when the taper only started at 3 km/h; the law v/0.65 only binds near the stop)
END_TAPER_LEAD = DELIVERY_LEAD  # s
END_TAPER_RELEASE_JERK = 3.0  # m/s^3, fastest the taper may lighten the brake (1.0 -> 3.0 with the earlier start: the cap now
                              # follows v/0.65 from 10 km/h, so it eases in instead of cutting at 3 km/h as on 00:07:28)


# The end taper softens the end of every stop, but never below what still stops the car short of a (nearly) stopped
# lead: device campaign 2026-10-02, 30 km/h, stopped car appearing 17 m ahead, PCM delivering 1:1: the planner asked
# -3.15 at 3.6 km/h with 0.46 m left and the taper capped it at -1.65; the stop ended 0.2 m from the lead (governor)
# or in contact (MPC alone). controlsd passes the deceleration that stops EMERGENCY_GAP behind the radar lead
# (x EMERGENCY_MARGIN for the PCM lag); a normal stop needs ~0.0-0.1 there, so its tuned end is unchanged.
EMERGENCY_GAP = 1.0     # m from the lead's rear
EMERGENCY_MARGIN = 1.2
EMERGENCY_LEAD_V = 1.0  # m/s: only a (nearly) stopped lead


def stop_decel_floor(v_ego: float, lead_present: bool, d_rel: float, v_lead: float) -> float:
  """Deceleration (positive) needed to stop EMERGENCY_GAP short of a nearly stopped lead; 0 when there is none."""
  if not lead_present or v_lead > EMERGENCY_LEAD_V or v_ego <= 0.0:
    return 0.0
  return EMERGENCY_MARGIN * v_ego ** 2 / (2.0 * max(d_rel - EMERGENCY_GAP, 0.3))


def end_taper_decel(v_ego: float, a_ego: float) -> float:
  """Largest deceleration (positive number) allowed this close to the stop."""
  v_future = max(v_ego + min(a_ego, 0.0) * END_TAPER_LEAD, 0.0)
  return end_decel(v_future)

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP_SP, active, long_control_state,
                             should_stop, brake_pressed, cruise_standstill):
  # Gas Interceptor
  cruise_standstill = cruise_standstill and not CP_SP.enableGasInterceptor

  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if should_stop:
        long_control_state = LongCtrlState.stopping

  return long_control_state

class LongControl:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.standstill_t = 0.0  # s of continuous CS.standstill
    self.stopping_t = 0.0  # s in the stopping state
    self.go_t = 0.0  # s the go condition has held while stopping at standstill
    self.stopped_once = False  # the wheels have stopped in this stopping episode
    self.creep_t = 0.0  # s of creep after that

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, has_lead=False, decel_floor=0.0):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    prev_state = self.long_control_state
    self.long_control_state = long_control_state_trans(self.CP_SP, active, self.long_control_state,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if prev_state == LongCtrlState.stopping and self.long_control_state == LongCtrlState.pid and CS.standstill:
      # standing still: only leave the hold once the go condition has held for STOPPING_EXIT_DEBOUNCE
      self.go_t += DT_CTRL
      if self.go_t < STOPPING_EXIT_DEBOUNCE:
        self.long_control_state = LongCtrlState.stopping
    else:
      self.go_t = 0.0
    self.standstill_t = self.standstill_t + DT_CTRL if CS.standstill else 0.0
    self.stopping_t = self.stopping_t + DT_CTRL if self.long_control_state == LongCtrlState.stopping else 0.0
    if self.long_control_state != LongCtrlState.stopping:
      self.stopped_once = False
    elif CS.standstill:
      self.stopped_once = True
    creeping = self.stopped_once and not CS.standstill and CS.vEgo > CREEP_V_MIN
    self.creep_t = self.creep_t + DT_CTRL if creeping else 0.0

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        # below stop_gap.FINAL_V the driver's fixed final request replaces the rolling minimum (2026-10-01 night)
        final = CS.vEgo < stop_gap_mod.FINAL_V
        follow_min = stop_gap_mod.FINAL_REQUEST if final else STOPPING_FOLLOW_MIN
        rolling_min = stop_gap_mod.FINAL_REQUEST if final else STOPPING_ROLLING_MIN
        if (not CS.standstill and not self.stopped_once and a_target <= follow_min and a_target > output_accel and
            self.stopping_t < STOPPING_FREEZE_MAX):
          # still rolling: keep easing with the plan's own end-of-stop curve (lighter only, rate-limited). Not once the
          # STOPPING_FREEZE_MAX safety net is ramping: following a lighter plan there undid the ramp every step and the
          # car settled where the request just balanced the creep torque, crawling on (device campaign 2026-10-02
          # without the stop-gap governor: -0.20 vs creep 0.20, 0.45 km/h into the lead)
          output_accel = min(a_target, rolling_min, output_accel + STOPPING_FOLLOW_RATE * DT_CTRL)
        if not CS.standstill and not self.stopped_once and output_accel > rolling_min:
          # never so light that creep torque holds the car rolling; ease up to the minimum at the hold rate
          output_accel = max(rolling_min, output_accel - STANDSTILL_HOLD_RATE * DT_CTRL)
        # TODO: can we just go straight to stopAccel?
        hold_delay = STANDSTILL_HOLD_DELAY_LEAD if has_lead else STANDSTILL_HOLD_DELAY_NO_LEAD
        if self.standstill_t >= hold_delay:
          rate = STANDSTILL_HOLD_RATE
        elif creeping:
          rate = min(CREEP_RATE_BASE + CREEP_RATE_GROWTH * self.creep_t, CREEP_RATE_MAX)
        elif self.stopping_t >= STOPPING_FREEZE_MAX:
          rate = STOPPING_DECEL_RATE
        else:
          rate = 0.0
        output_accel -= rate * DT_CTRL
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    if active and not CS.standstill and CS.vEgo < END_TAPER_V_START and a_target < 0.0 and not CS.brakePressed and \
       not (self.long_control_state == LongCtrlState.stopping and self.stopped_once):
      capped = max(output_accel, -max(end_taper_decel(CS.vEgo, CS.aEgo), decel_floor))
      if capped > output_accel:
        # never let the brake go faster than END_TAPER_RELEASE_JERK: a sudden release lets creep torque push the car on
        capped = min(capped, self.last_output_accel + END_TAPER_RELEASE_JERK * DT_CTRL)
        output_accel = max(output_accel, capped)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
