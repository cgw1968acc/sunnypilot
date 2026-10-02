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
  STANDSTILL_HOLD_RATE = 0.5  # m/s^2/s
  STANDSTILL_HOLD_DELAY_LEAD = 0.6  # s
  STANDSTILL_HOLD_DELAY_NO_LEAD = 0.9  # s
  STOPPING_FREEZE_MAX = 2.0  # s
  STOPPING_EXIT_DEBOUNCE = 0.2  # s
  STOPPING_FOLLOW_MIN = -0.10  # m/s^2
  STOPPING_FOLLOW_RATE = 2.0  # m/s^3
  CREEP_V_MIN = 0.03  # m/s
  CREEP_RATE_GROWTH = 1.5  # m/s^2/s per second of creep
  CREEP_RATE_MAX = 1.0  # m/s^2/s

  # The end of the stop, from the driver's own template (Corolla Altis Hybrid, 11 bookmarked manual stops on
  # 2026-10-01 11:40-11:54 and the 2026-09-30 22:01:54 one, measured with manual_stops.py): from ~3 km/h to the
  # moment the wheels stop the pedal stays CONSTANT at ~800-900 N (0.5-0.7 m/s^2 delivered), 3->1 km/h takes
  # 1.0-1.2 s, 1->0 km/h 0.3-0.5 s, no roll, no rebound (stops that ended above ~1100 N nodded). The driver never
  # lightens the pedal while still rolling; the firmer hold comes after standstill. So below END_V, while the plan
  # is braking to a stop, the request is held at END_REQUEST and never lighter, even when the planner's own curve
  # eases off; a harder planner request still passes (a late-seen lead), and a plan that wants to go (a_target > 0,
  # the lead moved off) releases it. END_REQUEST is the request that delivers the template force once the hybrid's
  # creep torque (~0.2 m/s^2 below 2.5 km/h) is taken off; confirm against the 0xA6 brake force on the car.
  END_V = 3.0 / 3.6  # m/s
  END_REQUEST = -0.65  # m/s^2
  END_PLAN_MIN = -0.25  # m/s^2: the plan must be braking this much to count as stopping (a crawl-follow hovers near 0)
  END_RATE = 2.0  # m/s^3: how fast the request firms to END_REQUEST on entry

  def __init__(self, stop_accel):
    self.stop_accel = stop_accel
    self.standstill_t = 0.0
    self.stopping_t = 0.0
    self.go_t = 0.0
    self.stopped_once = False
    self.creep_t = 0.0
    self.end_active = False

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
        floor = max(self.END_REQUEST, prev_accel - self.END_RATE * DT_CTRL)
        return state, float(np.clip(min(stock_accel, floor), accel_limits[0], accel_limits[1]))
      return state, stock_accel

    output_accel = prev_accel
    if output_accel > self.stop_accel:
      output_accel = min(output_accel, 0.0)
      if not CS.standstill and not self.stopped_once and a_target < self.STOPPING_FOLLOW_MIN and a_target > output_accel:
        output_accel = min(a_target, output_accel + self.STOPPING_FOLLOW_RATE * DT_CTRL)
      if self.end_active:
        # still rolling inside the end window: never lighter than the template force
        output_accel = min(output_accel, max(self.END_REQUEST, prev_accel - self.END_RATE * DT_CTRL))

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
