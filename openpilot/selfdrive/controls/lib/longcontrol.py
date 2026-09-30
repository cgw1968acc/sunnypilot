import numpy as np
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap import A_3KPH, A_2KPH, A_1KPH, A_0KPH, DELIVERY_LEAD

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

STOPPING_DECEL_RATE = 0.3  # m/s^2/s while trying to stop
# Once the car is actually standing still the brake request keeps falling towards stopAccel at this faster rate, so
# the hold firms up in well under a second instead of ~3 s (Altis 2026-09-27: after a stop behind a lead the car rolled
# on towards it). The approach and the stop itself are untouched: this rate only applies with CS.standstill set.
STANDSTILL_HOLD_RATE = 1.0  # m/s^2/s
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
STOPPING_FOLLOW_MIN = -0.10  # m/s^2
STOPPING_FOLLOW_RATE = 2.0  # m/s^3
# Universal end-of-stop taper (driver 2026-09-30 night: the 'taper' in the last 0.3 s is what makes a stop seamless, a
# human cannot repeat it every time, the code must). Whatever produced the request - the stop-gap governor, the lead
# MPC or an e2e stop with no lead - below END_TAPER_V_START the braking request may not exceed the tuned end-of-stop
# curve (the governor's A_3KPH..A_0KPH), read END_TAPER_LEAD ahead of the car's speed so the deceleration the PCM
# actually delivers (lagged) follows it. Positive targets pass, so a lead driving off is not held back.
END_TAPER_V_START = 3.0 / 3.6  # m/s
END_TAPER_LEAD = DELIVERY_LEAD  # s


def end_taper_decel(v_ego: float, a_ego: float) -> float:
  """Largest deceleration (positive number) allowed this close to the stop."""
  v_future = max(v_ego + min(a_ego, 0.0) * END_TAPER_LEAD, 0.0)
  return float(np.interp(v_future, [0.0, 1.0 / 3.6, 2.0 / 3.6, 3.0 / 3.6], [A_0KPH, A_1KPH, A_2KPH, A_3KPH]))

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

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, has_lead=False):
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

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        if not CS.standstill and a_target < STOPPING_FOLLOW_MIN and a_target > output_accel:
          # still rolling: keep easing with the plan's own end-of-stop curve (lighter only, rate-limited)
          output_accel = min(a_target, output_accel + STOPPING_FOLLOW_RATE * DT_CTRL)
        # TODO: can we just go straight to stopAccel?
        hold_delay = STANDSTILL_HOLD_DELAY_LEAD if has_lead else STANDSTILL_HOLD_DELAY_NO_LEAD
        if self.standstill_t >= hold_delay:
          rate = STANDSTILL_HOLD_RATE
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

    if active and not CS.standstill and CS.vEgo < END_TAPER_V_START and a_target < 0.0 and not CS.brakePressed:
      output_accel = max(output_accel, -end_taper_decel(CS.vEgo, CS.aEgo))

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
