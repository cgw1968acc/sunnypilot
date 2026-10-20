"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lead start assist: when the car is stopped behind a stopped lead and the lead starts to move, the stock lead MPC
takes close to a second before it asks for enough acceleration to leave the stopping state. Its distance cost is
scaled by 1 / (v_ego + 10) and the jerk cost dominates, so a lead that has only moved half a metre produces an
acceleration target below the 0.1 m/s^2 that should_stop() needs, and the Toyota PCM keeps the brakes on until the
request passes 0.3 m/s^2. In a drive-through lane the lead often moves one car length and stops again, so the car
does not move at all.

This module watches for the lead departure directly (raw lead speed or gap growth since the stop) and, once it is
seen, puts a floor under the planner's acceleration target for a short time. The MPC keeps control of everything
above that floor and takes over again as soon as the car is rolling, the gap stops growing, or the lead is lost.

tncr (Corolla Cross), owner 2026-10-20, route 00000093: "22:20 it stopped and then crept forward with the lead not moving
- remove that. Around then the lead pulled away quickly from a stop - why did we follow so slowly? Once stopped, hold
firm and do not move on a whim, but the moment the lead starts to move release the brake and accelerate at once by the
accel profile."
- 22:20:12.8: the stopped lead's radar speed sat at 0.15-0.30 m/s (gap flat at 3.0 m, filtered growth <= 0.05 m); the
  old speed-only trigger (0.25 m/s for 0.25 s) fired, the floor launched the car 0.9 m toward a lead that never moved.
- 22:20:58-22:21:01: the same noise kept the old arming (lead below 0.2 m/s for 0.2 s) from ever completing, so the
  assist never armed; the lead left (0.4 -> 3 m/s) and the stock MPC took 2 s to ask for +0.1, then +0.5..0.7.
So now: arming needs only ego stopped (any lead speed); the departure is the GAP really opening - filtered gap above
its minimum since the stop by GO_GAP with the lead speed at least CONFIRM_V, or a clear lead speed (STRONG_V) with the
gap at least not closing (STRONG_GAP) - held GO_HOLD. On route 93 this rejects 22:20:12 (no false start; the stands
there show <= 0.05 m of filtered gap noise) and fires 22:20:58.3, 2.3 s before the old plan did; the other four
launches fire 0.25-0.35 s later than the speed-only trigger did.
Once it fires the floor is no longer a gentle 0.35-0.5: it asks for the speed difference to the lead over T_MATCH,
between FLOOR_MIN and the accel profile's own maximum (a_max, from the planner: AccelPersonality, uphill factor), so a
lead that leaves briskly is followed briskly and one that creeps is followed gently; it hands back to the MPC once ego
has reached the lead's speed (the gap no longer grows), on a timeout, or if the gap closes.
"""
from collections import deque

import numpy as np

V_EGO_STOPPED = 0.3  # m/s, ego counts as stopped below this while arming
V_EGO_MAX = 20.0 / 3.6  # m/s, never active above this
LEAD_D_MIN = 2.0  # m, closest lead the assist will ever push towards (stops land at STOP_GAP = 3.0 m)
LEAD_D_MAX = 12.0  # m, farthest lead that counts as "the car we are queued behind"
ARM_TIME = 0.2  # s, ego stopped this long before the assist is armed (the lead may still be creeping)
LEAD_JUMP = 1.0  # m, a gap that closes this much in one frame is a different car (cut-in), start over
GAP_TAU = 0.2  # s, low-pass on the radar gap
GO_GAP = 0.15  # m, filtered gap growth since the stop that counts as a departure ...
CONFIRM_V = 0.2  # m/s ... with the lead speed at least this
STRONG_V = 0.45  # m/s, or a lead speed this clear (still-lead radar noise on route 93: <= 0.30) ...
STRONG_GAP = 0.05  # m ... with the gap at least not closing
GO_HOLD = 0.1  # s, the departure must hold this long
GROWTH_WINDOW = 1.5  # s: growth is measured from the smallest gap of this last window, so a slow drift over a long
                     # stand (route 93 22:20:16-59: 0.4 m in 40 s) never adds up to a departure
GAP_MIN_ACTIVE = 2.3  # m, release the floor if the (filtered) gap gets this small ...
GAP_MIN_START = 2.4  # m ... and start only from this (one radar dip must not cut a launch that just began)
LEAD_CLOSING_V_REL = -0.25  # m/s, release once ego is clearly faster than the lead
T_MATCH = 0.5  # s, floor = (lead speed - ego speed) / T_MATCH ...
FLOOR_MIN = 0.5  # m/s^2 ... at least this while ego is still slower than the lead ...
FLOOR_MAX_DEFAULT = 1.5  # m/s^2 ... and at most the accel profile's max (this when the planner gives none)
ACTIVE_TIMEOUT = 8.0  # s, hand back to the MPC no matter what after this long (normally ego reaching the lead speed ends it)


class LeadStartAssist:
  def __init__(self, dt: float):
    self.dt = dt
    self.reset()

  def reset(self) -> None:
    self.t_stopped = 0.0
    self.armed = False
    self.d_f: float | None = None
    self.d_stop = float('inf')
    self.d_hist: deque = deque()
    self.t_go = 0.0
    self.d_prev: float | None = None
    self.t_active: float | None = None

  @property
  def active(self) -> bool:
    return self.t_active is not None

  def update(self, allowed: bool, v_ego: float, lead_present: bool, d_rel: float, v_lead: float, v_rel: float,
             a_max: float | None = None) -> float | None:
    """Returns the acceleration floor to apply this cycle, or None when the assist has nothing to say.

    v_rel follows radarState: positive when the lead is faster than ego. a_max: the accel profile's limit."""
    lead_jumped = self.d_prev is not None and self.d_prev - d_rel > LEAD_JUMP
    if not allowed or not lead_present or not (LEAD_D_MIN <= d_rel <= LEAD_D_MAX) or lead_jumped or v_ego > V_EGO_MAX:
      self.reset()
      self.d_prev = d_rel if lead_present else None
      return None
    self.d_prev = d_rel
    self.d_f = d_rel if self.d_f is None else self.d_f + self.dt / (GAP_TAU + self.dt) * (d_rel - self.d_f)

    if self.t_active is not None:
      self.t_active += self.dt
      # release once ego has caught the lead's speed, timed out, the gap is too small or shrinking (pushing INTO a
      # lead that did not really depart)
      if (v_ego >= v_lead or self.t_active > ACTIVE_TIMEOUT or self.d_f < GAP_MIN_ACTIVE or
          v_rel < LEAD_CLOSING_V_REL or d_rel < self.d_stop - 0.3):
        self.reset()
        return None
      return self._floor(v_ego, v_lead, a_max)

    if v_ego >= V_EGO_STOPPED:
      self.t_stopped, self.armed, self.d_stop, self.t_go = 0.0, False, float('inf'), 0.0
      self.d_hist.clear()
      return None

    if not self.armed:
      self.t_stopped += self.dt
      self._track_gap()
      if self.t_stopped >= ARM_TIME - 1e-6:
        self.armed = True
      return None

    # armed: ego has stood, watch for the gap to really open
    self._track_gap()
    growth = self.d_f - self.d_stop
    departing = (growth >= GO_GAP and v_lead >= CONFIRM_V) or (v_lead >= STRONG_V and growth >= STRONG_GAP)
    self.t_go = self.t_go + self.dt if departing else 0.0
    if self.t_go >= GO_HOLD - 1e-6 and self.d_f >= GAP_MIN_START:
      self.t_active = 0.0
      return self._floor(v_ego, v_lead, a_max)
    return None

  def _track_gap(self) -> None:
    self.d_hist.append(self.d_f)
    while len(self.d_hist) > max(1, round(GROWTH_WINDOW / self.dt)):
      self.d_hist.popleft()
    self.d_stop = min(self.d_hist)

  @staticmethod
  def _floor(v_ego: float, v_lead: float, a_max: float | None) -> float:
    hi = FLOOR_MAX_DEFAULT if a_max is None else max(a_max, FLOOR_MIN)
    return float(np.clip((v_lead - v_ego) / T_MATCH, FLOOR_MIN, hi))
