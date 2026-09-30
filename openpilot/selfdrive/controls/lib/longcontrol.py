import numpy as np
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

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
STANDSTILL_HOLD_DELAY_LEAD = 0.8  # s (0.5 -> 0.8, driver 2026-09-30 second test: about 0.3 s later still)
STANDSTILL_HOLD_DELAY_NO_LEAD = 1.1  # s (0.8 -> 1.1)
# Before that the request is frozen at the value the glide ended on: the driver wants the stop itself exactly as it
# felt on the drives where the hold never ramped at all (stopAccel -0.02 days) and the extra force only once the
# car is at rest. Safety net: if the car has not reached standstill within STOPPING_FREEZE_MAX of entering the
# stopping state (request too light to finish the stop, e.g. a creep), the upstream STOPPING_DECEL_RATE ramp resumes.
STOPPING_FREEZE_MAX = 2.0  # s

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

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, has_lead=False):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP_SP, active, self.long_control_state,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    self.standstill_t = self.standstill_t + DT_CTRL if CS.standstill else 0.0
    self.stopping_t = self.stopping_t + DT_CTRL if self.long_control_state == LongCtrlState.stopping else 0.0

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
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

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
