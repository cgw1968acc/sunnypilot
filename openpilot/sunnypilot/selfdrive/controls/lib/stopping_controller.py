"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState


class StoppingController:
  """Optional terminal-stop policy applied after stock LongControl.update()."""

  STOPPING_DECEL_RATE = 0.3  # m/s^2/s
  # once the wheels really stand (wheel pulse) and the hold delay has passed, press on firmly to stopAccel: the driver
  # presses to 1000-2400 N within 1-2 s of stopping (owner 2026-10-02: the car must not be able to move again, also on
  # a slope). 1.0 m/s^2/s reaches the default -2.0 (~2900 N) in ~1.9 s; the old 0.5 took ~3.7 s.
  STANDSTILL_HOLD_RATE = 1.0  # m/s^2/s
  STANDSTILL_HOLD_DELAY_LEAD = 0.6  # s
  STANDSTILL_HOLD_DELAY_NO_LEAD = 0.9  # s
  STOPPING_FREEZE_MAX = 2.0  # s
  STOPPING_EXIT_DEBOUNCE = 0.2  # s
  STOPPING_FOLLOW_MIN = -0.10  # m/s^2
  STOPPING_FOLLOW_RATE = 2.0  # m/s^3
  CREEP_V_MIN = 0.03  # m/s
  CREEP_RATE_GROWTH = 1.5  # m/s^2/s per second of creep
  CREEP_RATE_MAX = 1.0  # m/s^2/s

  # The end of the stop, from the driver's own manual stops (Corolla Altis Hybrid, 42 stops over 2026-09-30..10-02
  # measured with manual_stops.py / lightest_stop.py): from ~3 km/h to the moment the wheels stop the pedal stays
  # CONSTANT - the driver never lightens it while rolling and presses firmer only after standstill - and the car still
  # stops without rolling on. The lightest pedal force that stopped the car and held it: 640 N (2 of 3 held), 680 N and
  # above held every time (5 of 5 at 720 N); 600 N and lighter crept on (the hybrid's creep torque balances the brake at
  # ~620 N). Stops that ended above ~1100 N nodded (IMU rebound 0.4-1.5). The PCM's settled brake force below 3 km/h
  # follows BF ~ 480 + 1200 * |request| N (op stops, 0xA6 BRAKE_FORCE: -0.3 -> ~800 N, -0.5 -> ~1080 N, -1.0 ->
  # ~1840 N), so END_REQUEST = -0.15 asks for ~660 N: the lightest force that still stops and holds. Below END_V, while
  # the plan is braking to a stop, the request is HELD at END_REQUEST - never lighter (creep) and never firmer (nod),
  # even when the planner's own curve eases off or firms up; a hard plan (END_HARD, a late-seen lead) still passes,
  # and a plan that wants to go (a_target > 0, the lead moved off) releases it. After standstill the hold ramp to
  # stopAccel applies unchanged.
  END_V = 3.0 / 3.6  # m/s
  END_REQUEST = -0.15  # m/s^2 (~660 N)
  END_HARD = -1.5  # m/s^2: a plan at least this hard is an emergency and passes through
  END_PLAN_MIN = -0.25  # m/s^2: the plan must be braking this much to count as stopping (a crawl-follow hovers near 0)
  END_RATE = 2.0  # m/s^3: how fast the request moves to END_REQUEST on entry

  def __init__(self, stop_accel):
    self.stop_accel = stop_accel
    self.standstill_t = 0.0
    self.stopping_t = 0.0
    self.go_t = 0.0
    self.stopped_once = False
    self.creep_t = 0.0
    self.end_active = False

  def _end_request(self, a_target, prev_accel):
    target = a_target if a_target <= self.END_HARD else self.END_REQUEST
    step = self.END_RATE * DT_CTRL
    return float(np.clip(target, prev_accel - step, prev_accel + step))

  def update(self, prev_state, state, CS, a_target, prev_accel, stock_accel, accel_limits, has_lead=False):
    if prev_state == LongCtrlState.stopping and state == LongCtrlState.pid and CS.standstill:
      self.go_t += DT_CTRL
      if self.go_t < self.STOPPING_EXIT_DEBOUNCE:
        state = LongCtrlState.stopping
    else:
      self.go_t = 0.0

    self.standstill_t = self.standstill_t + DT_CTRL if CS.standstill else 0.0
    self.stopping_t = self.stopping_t + DT_CTRL if state == LongCtrlState.stopping else 0.0
    if state != LongCtrlState.stopping:
      self.stopped_once = False
    elif CS.standstill:
      self.stopped_once = True

    creeping = self.stopped_once and not CS.standstill and CS.vEgo > self.CREEP_V_MIN
    self.creep_t = self.creep_t + DT_CTRL if creeping else 0.0

    # end-of-stop window: rolling below END_V with a plan that is braking to a stop
    rolling = not CS.standstill and not self.stopped_once
    if state == LongCtrlState.off or not rolling or CS.vEgo >= self.END_V or a_target > 0.0:
      self.end_active = False
    elif a_target <= self.END_PLAN_MIN:
      self.end_active = True

    if state != LongCtrlState.stopping:
      if self.end_active:
        return state, float(np.clip(self._end_request(a_target, prev_accel), accel_limits[0], accel_limits[1]))
      return state, stock_accel

    output_accel = prev_accel
    if output_accel > self.stop_accel:
      output_accel = min(output_accel, 0.0)
      if not CS.standstill and not self.stopped_once and a_target < self.STOPPING_FOLLOW_MIN and a_target > output_accel:
        output_accel = min(a_target, output_accel + self.STOPPING_FOLLOW_RATE * DT_CTRL)
      if self.end_active:
        # still rolling inside the end window: the template force, whatever the plan's own curve does
        output_accel = self._end_request(a_target, prev_accel)

      hold_delay = self.STANDSTILL_HOLD_DELAY_LEAD if has_lead else self.STANDSTILL_HOLD_DELAY_NO_LEAD
      if self.standstill_t >= hold_delay:
        rate = self.STANDSTILL_HOLD_RATE
      elif creeping:
        rate = min(self.STOPPING_DECEL_RATE + self.CREEP_RATE_GROWTH * self.creep_t, self.CREEP_RATE_MAX)
      elif self.stopping_t >= self.STOPPING_FREEZE_MAX:
        rate = self.STOPPING_DECEL_RATE
      else:
        rate = 0.0
      output_accel -= rate * DT_CTRL

    return state, float(np.clip(output_accel, accel_limits[0], accel_limits[1]))
