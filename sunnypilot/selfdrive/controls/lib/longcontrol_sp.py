"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, long_control_state_trans, LongCtrlState

interp = np.interp

# Floor for the stopping ramp to limit how negative accel winds during a long hold.
# Shallower floor means less ramp to unwind on departure, smoother pull-away.
STOPPING_ACCEL_FLOOR = -0.25

# Rise rate when leaving stopping state, applied until ramp recovers to ~0.
STOPPING_EXIT_RELEASE_RATE = 1.5  # m/s²/s

# Speed-dependent jerk caps on a_target feedforward.
# Decel is tight at low speed for smooth stops, looser at highway so the car
# responds promptly to a hard-braking lead or FCW event.
# Accel is moderate at all speeds to smooth tip-in without lagging cruise resumption.
A_TARGET_JERK_ACCEL_BP = [0.0, 10.0, 20.0]         # m/s
A_TARGET_JERK_ACCEL_V  = [1.5,  2.5,  4.0]         # m/s³
A_TARGET_JERK_DECEL_BP = [0.0,  5.0, 10.0, 20.0, 30.0]  # m/s
A_TARGET_JERK_DECEL_V  = [3.0,  4.0,  6.0, 10.0, 15.0]  # m/s³


def _enter_stop_target(v_ego: float, a_target: float, dist: float | None) -> bool:
  if dist is None or dist <= 0.0:
    return False
  min_d = float(np.clip(
    interp(v_ego,     [0.0, 1.0, 2.3, 4.2, 6.0], [0.20, 0.22, 0.34, 0.44, 0.48]) -
    interp(-a_target, [0.2, 0.6, 1.2, 1.8],       [0.0,  0.08, 0.22, 0.30]),
    0.2, 0.5,
  ))
  if dist <= min_d:
    return False
  d = float(np.clip(dist, 0.0, 6.0))
  return (d < interp(v_ego, [0.0, 0.6, 1.5, 3.0, 5.0], [0.35, 0.65, 1.10, 1.70, 2.30]) and
          a_target < interp(v_ego, [0.0, 0.6, 1.5, 3.0, 5.0], [-0.03, -0.06, -0.10, -0.16, -0.22]))


def _hold_stop_target(v_ego: float, a_target: float, dist: float | None) -> bool:
  if _enter_stop_target(v_ego, a_target, dist):
    return True
  if dist is None or dist <= 0.0:
    return False
  return (v_ego < 2.5 and
          dist < interp(v_ego, [0.0, 0.8, 1.5, 2.4], [1.80, 2.10, 2.40, 2.80]) and
          a_target < interp(v_ego, [0.0, 0.8, 1.5, 2.4], [-0.03, -0.05, -0.08, -0.12]))


class LongControlSP(LongControl):

  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self._was_off                   = True
    self._stopping_exit_slew_active = False
    self._a_target_slewed           = 0.0

  def reset(self):
    super().reset()
    self._was_off                   = True
    self._stopping_exit_slew_active = False
    # _a_target_slewed is not reset here; it is kept in sync with aEgo during
    # non-pid states so the slew reference is already correct on re-entry to pid.

  def update(self, active, CS, a_target, should_stop, accel_limits,
             distance_to_stop_target_m=None, **kwargs):
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    dist = distance_to_stop_target_m

    stop_target = _enter_stop_target(CS.vEgo, a_target, dist)
    effective_should_stop = should_stop or stop_target
    new_state = long_control_state_trans(
      self.CP, self.CP_SP, active, self.long_control_state, CS.vEgo,
      effective_should_stop, CS.brakePressed, CS.cruiseState.standstill,
    )

    leaving_stopping = self.long_control_state == LongCtrlState.stopping and new_state != LongCtrlState.stopping
    if leaving_stopping and effective_should_stop and _hold_stop_target(CS.vEgo, a_target, dist):
      new_state = LongCtrlState.stopping
      leaving_stopping = False

    if leaving_stopping and new_state in (LongCtrlState.pid, LongCtrlState.starting):
      self._stopping_exit_slew_active = self.last_output_accel < -0.10

    self.long_control_state = new_state

    output_accel = self.last_output_accel

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      self._a_target_slewed = float(np.clip(CS.aEgo, accel_limits[0], accel_limits[1]))
      output_accel = 0.0

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = min(output_accel, 0.0)
      if output_accel > self.CP.stopAccel:
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      if CS.vEgo < 0.3:
        output_accel = max(output_accel, STOPPING_ACCEL_FLOOR)
      self._a_target_slewed = float(np.clip(CS.aEgo, accel_limits[0], accel_limits[1]))
      self.pid.reset()
      self._was_off = False
      self._stopping_exit_slew_active = False

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self._a_target_slewed = float(np.clip(CS.aEgo, accel_limits[0], accel_limits[1]))
      self.pid.reset()
      self._was_off = False
      self._stopping_exit_slew_active = False

    else:
      was_off = self._was_off
      self._was_off = False
      if was_off:
        self._a_target_slewed = float(np.clip(CS.aEgo, accel_limits[0], accel_limits[1]))
        self.last_output_accel = self._a_target_slewed
        output_accel = self.last_output_accel
        self.pid.reset()
      else:
        jerk_decel = interp(CS.vEgo, A_TARGET_JERK_DECEL_BP, A_TARGET_JERK_DECEL_V)
        jerk_accel = interp(CS.vEgo, A_TARGET_JERK_ACCEL_BP, A_TARGET_JERK_ACCEL_V)
        self._a_target_slewed = float(np.clip(
          a_target,
          self._a_target_slewed - jerk_decel * DT_CTRL,
          self._a_target_slewed + jerk_accel * DT_CTRL,
        ))
        output_accel = self.pid.update(self._a_target_slewed - CS.aEgo, speed=CS.vEgo, feedforward=self._a_target_slewed)
      if self._stopping_exit_slew_active:
        output_accel = min(output_accel, self.last_output_accel + STOPPING_EXIT_RELEASE_RATE * DT_CTRL)
        if self.last_output_accel >= -0.05:
          self._stopping_exit_slew_active = False

    self.last_output_accel = float(np.clip(output_accel, accel_limits[0], accel_limits[1]))
    return self.last_output_accel
