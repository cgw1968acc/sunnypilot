"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.realtime import DT_CTRL

# Stop-gap close. Owner 2026-10-14 evening (route 0000015c, true GPS time 19:35:40): the stop behind a stopped lead
# ended 5.5 m out (radar dRel; the plan itself aimed at ~5.4 m), and most stops that evening ended 4.3-6.4 m out.
# Owner: "I do not want the car to stop and then creep forward a second time. The system should predict that it will
# stop beyond 3.5 m, not let the car stop completely, and keep creeping" up to 3-3.5 m; "keep the speed steady, do not
# over-release so it goes fast-slow-fast"; and (owner's end-force rule) all force modulation happens above 2 km/h, below
# it only a linear taper to -0.30 at 0 km/h (end_force_limit.py, the last stage, does that for every stop).
# That is step 5 of the owner's braking principle (released too little, would stop too far -> release more).
# So: below V_MAX, behind a stopped lead, once the predicted stop gap is beyond GAP_TRIGGER even with the WEAKER of
# the request and the delivered decel (capped at B_PRED_MAX: the end of a stop is lighter than its middle), the request
# becomes the constant decel that ends at the target point, recomputed every frame from the filtered speed and radar
# distance: b = v^2 / 2(d - GAP_TARGET - TAPER_ROLL). It is self-correcting without any integrator - if the car slows
# too much b falls and it rolls on (never stops short, never a second launch), if it rolls too fast b rises - so the
# speed only falls, steadily, to the target. The request for b includes what the hybrid's creep torque eats at low
# speed (CREEP_*), corrected by a proportional decel feedback. The change into it and every change after is rate limited (FIRM_RATE / RELEASE_RATE, linear, no
# bite). It hands back to the plain request (at RETURN_RATE) when the lead moves, the radar lead drops or jumps, the plan
# stops braking, or the stop can no longer be made lightly (b > B_MAX and firmer than the plan). Only actuators.accel is
# shaped (same rule as stop_gap_hold).
V_MAX = 12.0 / 3.6         # m/s: decides only in the last part of the stop
LEAD_V_MAX = 0.5           # m/s: lead treated as stopped
GAP_TRIGGER = 4.0          # m (radar dRel): predicted stop beyond this -> close the gap
GAP_TARGET = 3.1           # m: aim of the final stop (owner: 3-3.5 m; the sim ends ~0.2 m beyond it)
TAPER_ROLL = 0.25          # m: the 2 -> 0 km/h taper to -0.30 (end_force_limit) rolls a little further: aim short of it
B_MIN = 0.3                # m/s^2: floor of the decel used in the prediction
B_PRED_MAX = 0.9           # m/s^2: and its cap (route 00000159 09:14:41: -1.14 at 12 km/h and 9.3 m predicted 4.4 m,
                           # the stop ended 2.8 m)
HARD_ACCEL = -2.0          # m/s^2: never during a hard brake (route 00000159 08:58:18, -3.5 at 6.7 km/h)
BRAKING_T = 0.5            # s: braking (request < 0) for this long before it may act (not right after engaging)
B_MAX = 0.8                # m/s^2: firmest decel it asks for by itself
ROOM_MIN = 0.1             # m
K_A = 1.0                  # (m/s^2)/(m/s^2): decel feedback
ROOM_END = 0.4             # m: this close to the target point ...
V_END = 0.7 / 3.6          # m/s: ... or this slow, the request is at least END_ACCEL - the owner's end of a stop
END_ACCEL = -0.35          # m/s^2 (tncr, Corolla Cross end of stop; Altis -0.30)
D_FILTER_T = 0.3           # s: radar distance filter
V_FILTER_T = 0.3           # s: speed filter
# request for a decel at low speed: the hybrid's creep torque eats the first part of the request (route 00000159:
# BRAKE_FORCE ~ 480 + 1200 * |req| N against ~750 N of creep -> decel ~ 1.2 * |req| - 0.27 below ~3 km/h; -0.55 gave
# -0.44, -0.30 ~0); above ~6 km/h the PCM gives what it is asked
CREEP_V = [3.0 / 3.6, 6.0 / 3.6]  # m/s
CREEP_OFF = [0.27, 0.0]           # m/s^2
CREEP_GAIN = [1.2, 1.0]
RELEASE_RATE = 1.0         # m/s^3: lighter (owner: linear release)
FIRM_RATE = 1.0            # m/s^3: firmer (no bite)
RETURN_RATE = 1.5          # m/s^3: back to the plain request after a hand-back
JUMP_D = 1.0               # m: a dRel step this large in one frame = a different / flickering lead
LEAD_LOST_T = 0.3          # s


def decel_request(b: float, v_ego: float) -> float:
  return -(b + float(np.interp(v_ego, CREEP_V, CREEP_OFF))) / float(np.interp(v_ego, CREEP_V, CREEP_GAIN))


class StopGapClose:
  def __init__(self):
    self.reset()

  def reset(self):
    self.active = False
    self.out = None          # shaped output while active / returning
    self.braking_t = 0.0
    self.lost_t = 0.0
    self.last_d = None
    self.v_f = None
    self.d_f = None
    self.a_f = None

  def _hand_back(self):
    self.active, self.v_f, self.d_f, self.a_f = False, None, None, None

  def update(self, accel: float, v_ego: float, a_ego: float, standstill: bool, lead, long_active: bool) -> float:
    if not long_active or standstill:
      self.reset()
      return accel

    self.braking_t = self.braking_t + DT_CTRL if accel < 0.0 else 0.0
    present = bool(lead.present)
    self.lost_t = 0.0 if present else self.lost_t + DT_CTRL
    d = lead.dRel if present else None
    jumped = d is not None and self.last_d is not None and abs(d - self.last_d) > JUMP_D
    if d is not None:
      self.last_d = d

    if self.active:
      if self.lost_t > LEAD_LOST_T or jumped or (present and lead.vLead >= LEAD_V_MAX) or accel >= 0.0:
        self._hand_back()
    elif present and lead.vLead < LEAD_V_MAX and not jumped and v_ego <= V_MAX and self.braking_t >= BRAKING_T and \
         accel > HARD_ACCEL:
      b = float(np.clip(min(-accel, -a_ego), B_MIN, B_PRED_MAX))
      if d - v_ego ** 2 / (2.0 * b) > GAP_TRIGGER:
        self.active = True
        self.out = accel

    if self.active:
      self.v_f = v_ego if self.v_f is None else self.v_f + (v_ego - self.v_f) * DT_CTRL / V_FILTER_T
      self.d_f = self.last_d if self.d_f is None else self.d_f + (self.last_d - self.d_f) * DT_CTRL / D_FILTER_T
      b = self.v_f ** 2 / (2.0 * max(self.d_f - GAP_TARGET - TAPER_ROLL, ROOM_MIN))
      # decel feedback (proportional, no integrator): the creep torque the request mapping assumes differs from car to
      # car and moment to moment - decelerating more than b lightens, less than b firms
      self.a_f = a_ego if self.a_f is None else self.a_f + (a_ego - self.a_f) * DT_CTRL / V_FILTER_T
      target = decel_request(b, v_ego) + K_A * (-b - self.a_f)
      if b > B_MAX and accel < target:  # can no longer be done lightly and the plan brakes firmer: it takes over
        self._hand_back()
      else:
        target = min(max(target, decel_request(B_MAX, v_ego)), 0.0)
        if self.d_f - GAP_TARGET < ROOM_END or self.v_f < V_END:  # the stop itself: at least the owner's end -0.30
          target = min(target, END_ACCEL)
        self.out = float(np.clip(target, self.out - FIRM_RATE * DT_CTRL, self.out + RELEASE_RATE * DT_CTRL))
        return self.out

    if self.out is not None:            # returning to the plain request after a hand-back
      self.out = float(np.clip(accel, self.out - RETURN_RATE * DT_CTRL, self.out + RETURN_RATE * DT_CTRL))
      if abs(self.out - accel) < 1e-3:
        self.out = None
        return accel
      return self.out
    return accel
