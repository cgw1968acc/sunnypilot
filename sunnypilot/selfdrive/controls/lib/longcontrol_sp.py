"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, long_control_state_trans, LongCtrlState
from openpilot.selfdrive.controls.lib.stop_and_go_helpers import should_release_stop_hold_for_departing_lead
from openpilot.selfdrive.controls.lib.stopping_guard import apply_low_speed_output_slew

interp = np.interp

STOPPING_V_BP      = [0.01,  0.2,  0.5]
STOPPING_ACCEL_MAX = [-0.01, -0.1, -0.3]

# Floor for the stopping ramp to limit how negative accel winds during a long hold.
# Shallower floor means less ramp to unwind on departure faster, smoother pull-away.
STOPPING_ACCEL_FLOOR = -0.25

# Rise rate when leaving stopping state, applied until ramp recovers to ~0.
STOPPING_EXIT_RELEASE_RATE = 1.5  # m/s²/s


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
          dist < interp(v_ego, [0.0, 0.8, 1.5, 2.4], [1.20, 1.40, 1.65, 1.90]) and
          a_target < interp(v_ego, [0.0, 0.8, 1.5, 2.4], [-0.04, -0.07, -0.10, -0.16]))


def _approach_active(v_ego: float, a_target: float, dist: float | None) -> bool:
  if _enter_stop_target(v_ego, a_target, dist) or dist is None or dist <= 0.0:
    return False
  d = float(np.clip(dist, 0.0, 6.0))
  return (v_ego > 1.0 and
          d < interp(v_ego, [1.0, 2.8, 4.5, 7.0], [1.0, 3.0, 3.8, 4.8]) and
          a_target < interp(v_ego, [1.0, 2.8, 4.5, 7.0], [-0.04, -0.07, -0.10, -0.14]))


def _carry_active(v_ego: float, a_target: float, dist: float | None) -> bool:
  if _enter_stop_target(v_ego, a_target, dist) or _approach_active(v_ego, a_target, dist):
    return False
  if dist is None or dist <= 0.0 or not (0.55 < v_ego < 1.25) or a_target > -0.08:
    return False
  requested_decel = float(np.clip(-a_target, 0.12, 0.90))
  predicted_stop  = (v_ego * v_ego) / max(2.0 * requested_decel, 0.24)
  return dist > (predicted_stop + interp(v_ego, [0.55, 0.85, 1.25], [0.80, 1.05, 1.35]))


def _approach_accel_cap(v_ego: float, dist: float | None) -> float:
  d = float(np.clip(0.0 if dist is None else dist, 0.0, 6.0))
  return min(
    interp(d,     [0.6, 1.0, 1.6, 2.4, 3.5], [-0.26, -0.22, -0.17, -0.12, -0.08]),
    interp(v_ego, [1.0, 2.8, 4.5, 7.0],       [-0.08, -0.14, -0.20, -0.26]),
  )


def _carry_accel_floor(v_ego: float, dist: float | None) -> float:
  d = float(np.clip(0.0 if dist is None else dist, 0.0, 6.0))
  return max(
    interp(d,     [1.4, 2.2, 3.2, 4.5, 6.0], [-0.34, -0.30, -0.26, -0.22, -0.18]),
    interp(v_ego, [0.55, 0.75, 0.95, 1.25],   [-0.34, -0.30, -0.26, -0.22]),
  )


def _dropout_guard(
  v_ego: float,
  a_target: float | None,
  dist: float | None,
  last_dist: float | None,
  last_accel: float,
  time_since_stop_s: float,
) -> bool:
  if a_target is None or dist is None or dist <= 0.0:
    return False
  if last_dist is None or last_dist <= 0.0:
    return False
  if not (0.0 < v_ego < 0.22) or last_accel > -0.12 or time_since_stop_s > 0.35:
    return False
  if dist > interp(v_ego, [0.00, 0.08, 0.16, 0.22], [1.05, 0.98, 0.92, 0.86]):
    return False
  if dist > last_dist + interp(v_ego, [0.00, 0.08, 0.16, 0.22], [0.06, 0.08, 0.10, 0.12]):
    return False
  return a_target <= interp(v_ego, [0.00, 0.08, 0.16, 0.22], [0.30, 0.26, 0.20, 0.14]) + 1e-6


class LongControlSP(LongControl):

  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.time_since_stop_intent_s   = 10.0
    self.time_since_standstill_s    = 10.0
    self.last_dist: float | None    = None
    self._release_lock_counter      = 0
    self._was_off                   = True
    self._stopping_exit_slew_active = False

  def reset(self):
    super().reset()
    self.time_since_stop_intent_s   = 10.0
    self.time_since_standstill_s    = 10.0
    self.last_dist                  = None
    self._release_lock_counter      = 0
    self._was_off                   = True
    self._stopping_exit_slew_active = False

  def update(self, active, CS, a_target, should_stop, accel_limits,
             distance_to_stop_target_m=None, lead_status=False, lead_v=0.0, lead_d_rel=0.0):
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    prev_dist = self.last_dist
    dist      = distance_to_stop_target_m

    stop_target = _enter_stop_target(CS.vEgo, a_target, dist)
    stop_active = should_stop or stop_target
    approach    = not stop_active and _approach_active(CS.vEgo, a_target, dist)
    carry       = not stop_active and not approach and _carry_active(CS.vEgo, a_target, dist)

    standstill = bool(getattr(CS, 'standstill', False)) or bool(CS.cruiseState.standstill)
    departing  = should_release_stop_hold_for_departing_lead(
      output_should_stop=bool(should_stop),
      standstill=standstill,
      v_ego=float(CS.vEgo),
      v_ego_starting=float(self.CP.vEgoStarting),
      lead_status=bool(lead_status),
      lead_v=float(lead_v),
      lead_d_rel=float(lead_d_rel),
    )
    if departing:
      stop_active = False
      approach    = False

    effective_should_stop = (should_stop or stop_target) and not departing
    new_state = long_control_state_trans(
      self.CP, self.CP_SP, active, self.long_control_state, CS.vEgo,
      effective_should_stop, CS.brakePressed, CS.cruiseState.standstill,
    )

    leaving_stopping = self.long_control_state == LongCtrlState.stopping and new_state != LongCtrlState.stopping
    if leaving_stopping:
      if effective_should_stop and _hold_stop_target(CS.vEgo, a_target, dist):
        new_state = LongCtrlState.stopping
        leaving_stopping = False
      elif _dropout_guard(CS.vEgo, a_target, dist, prev_dist, self.last_output_accel, self.time_since_stop_intent_s):
        new_state = LongCtrlState.stopping
        leaving_stopping = False

    if leaving_stopping and new_state in (LongCtrlState.pid, LongCtrlState.starting):
      self._stopping_exit_slew_active = self.last_output_accel < -0.10

    self.long_control_state = new_state

    if standstill:
      self.time_since_standstill_s = 0.0
    else:
      self.time_since_standstill_s = min(self.time_since_standstill_s + DT_CTRL, 10.0)

    if stop_active or approach or carry or self.long_control_state == LongCtrlState.stopping:
      self.time_since_stop_intent_s = 0.0
    else:
      self.time_since_stop_intent_s = min(self.time_since_stop_intent_s + DT_CTRL, 10.0)

    max_expected_accel = interp(CS.vEgo, STOPPING_V_BP, STOPPING_ACCEL_MAX)
    if 0.002 < CS.vEgo < 1.2 and self.last_output_accel < -0.05 and (CS.aEgo - max_expected_accel) >= 0.04:
      self._release_lock_counter = max(
        self._release_lock_counter,
        int(interp(CS.vEgo, [0.0, 0.20, 0.60, 1.20], [110, 95, 70, 50]) * DT_CTRL / 0.01)
      )
    elif self._release_lock_counter > 0:
      self._release_lock_counter -= 1
    release_lock = self._release_lock_counter > 0

    output_accel = self.last_output_accel

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.0

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = min(output_accel, 0.0)
      if output_accel > self.CP.stopAccel:
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      output_accel = max(output_accel, STOPPING_ACCEL_FLOOR)
      self.pid.reset()
      self._was_off = False
      self._stopping_exit_slew_active = False

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.pid.reset()
      self._was_off = False
      self._stopping_exit_slew_active = False

    else:
      if self._was_off:
        # Seed from aEgo so the PID doesn't start with a large error on engagement.
        self.last_output_accel = float(np.clip(CS.aEgo, accel_limits[0], accel_limits[1]))
        output_accel = self.last_output_accel
      self._was_off = False
      output_accel = self.pid.update(a_target - CS.aEgo, speed=CS.vEgo, feedforward=a_target,
                                     freeze_integrator=approach or carry)
      if approach:
        output_accel = min(output_accel, _approach_accel_cap(CS.vEgo, dist))
      if carry:
        output_accel = max(output_accel, _carry_accel_floor(CS.vEgo, dist))
      if self._stopping_exit_slew_active:
        output_accel = min(output_accel, self.last_output_accel + STOPPING_EXIT_RELEASE_RATE * DT_CTRL)
        if self.last_output_accel >= -0.05:
          self._stopping_exit_slew_active = False

    if self.long_control_state != LongCtrlState.off:
      allow_fast_release = (
        not stop_active and not approach and
        self.long_control_state in (LongCtrlState.pid, LongCtrlState.starting) and
        a_target > 0.2 and CS.vEgo > 0.12
      )
      if departing:
        allow_fast_release = True
      if self.time_since_stop_intent_s < 1.0 and self.time_since_standstill_s >= 0.5 and stop_active:
        allow_fast_release = False

      if not (self.long_control_state == LongCtrlState.stopping and stop_active):
        output_accel = apply_low_speed_output_slew(
          output_accel=output_accel,
          last_output_accel=self.last_output_accel,
          should_stop=stop_active or approach or carry,
          v_ego=CS.vEgo,
          a_ego=CS.aEgo,
          max_expected_accel=max_expected_accel,
          allow_fast_release=allow_fast_release,
          release_lock_active=release_lock,
        )

    self.last_dist = float(dist) if dist is not None and dist > 0.0 else None
    self.last_output_accel = float(np.clip(output_accel, accel_limits[0], accel_limits[1]))
    return self.last_output_accel
