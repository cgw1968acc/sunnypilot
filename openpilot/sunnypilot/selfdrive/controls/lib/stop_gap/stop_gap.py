"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Low-speed lead follow with a repeatable gap. The lead MPC's distance cost is soft, so behind a stopped or
crawling lead it neither settles at a consistent distance (2026-09-20 rlogs: stops at 2.98-4.35 m) nor follows a
creeping lead smoothly (it carried the car to 0.6 m/s for a lead that only nudged 0.4 m, then braked at -2.0 to
re-stop 0.9 m closer, a stop-move-slam cycle).

Once the lead is stopped or crawling (|vLead| < CREEP_LEAD_V) and ego is below V_ENGAGE within the
distance where a comfortable constant stop is due, this governs the closing speed toward a point STOP_GAP metres behind the lead. It works in the closing frame:
the deceleration that brings the closing speed (v_ego - vLead) to zero exactly at that point, v_close^2 / (2 s),
re-planned every frame. So a stopped lead is stopped behind at STOP_GAP, and a lead creeping at vLead is followed at
STOP_GAP with the closing speed bled off gently instead of overshooting and slamming.

It only ever adds braking: the returned target is min(this demand, the MPC's own target), so it can tighten a stop
but never brakes less than the MPC for a genuinely decelerating lead, and it hands back (returns None) the moment
the lead is opening faster than ego (a real pull-away, left to the MPC and the lead start assist). At and below
the release from the MPC's
target at engage is rate limited so taking over never lets the brake go with a jolt.
"""
import numpy as np

STOP_GAP = 3.0  # m, radar distance to the lead at the stop
V_ENGAGE = 8.5  # m/s, ego speed below which the governor takes over the stop from the MPC
A_ENGAGE = 1.0  # m/s^2, the governor starts braking when a constant stop at this deceleration is due, so a faster
                # approach begins shedding speed earlier (in the 8-3 m/s band) instead of braking hard late
S_ENGAGE_BASE = 3.0  # m, plus the stopping distance at A_ENGAGE
S_ENGAGE_MAX = 50.0  # m, hard cap on how far out it will engage
CREEP_LEAD_V = 2.0  # m/s, lead speed below which the governor manages the follow (above this the MPC does)
LEAD_STOPPED_V = 0.25  # m/s, lead speed below which it counts as fully stopped (min brake + parking hold apply)
HANDBACK_V_REL = 0.2  # m/s, lead opening faster than ego by this much is a pull-away: hand back to the MPC
A_MIN = 0.12  # m/s^2, least deceleration while closing on a stopped lead: gentle, so the last metres glide in
A_PAST = 1.0  # m/s^2, firm (not full-force) brake once past the point, so a small overshoot does not slam
A_NEG_MAX = -2.0  # m/s^2, hardest braking the governor asks for (a cut-in closer than that is the MPC's problem)
S_MIN = 0.1  # m, floor on the distance used in v_close^2 / (2 s)
TAU_CLOSE = 0.6  # s, how quickly the closing-speed error is turned into deceleration when not on the stopping curve
RELEASE_RATE = 1.0  # m/s^3, fastest the demand may get lighter, from the MPC's target at engage
DISENGAGE_MARGIN = 1.0  # hysteresis on V_ENGAGE and S_ENGAGE once engaged


class StopGapGovernor:
  def __init__(self, dt: float):
    self.dt = dt
    self.engaged = False
    self.a_prev = 0.0

  def reset(self) -> None:
    self.engaged = False
    self.a_prev = 0.0

  def update(self, allowed: bool, v_ego: float, lead_present: bool, d_rel: float, v_lead: float, a_current: float) -> float | None:
    """Acceleration target for the low-speed follow, or None when the MPC should keep control.
    a_current is the MPC's target this frame; the governor never brakes less than it, and releases from it slowly."""
    s = d_rel - STOP_GAP
    lead_slow = lead_present and v_lead < CREEP_LEAD_V
    v_close = v_ego - max(v_lead, 0.0)
    if not allowed or not lead_slow or v_close < -HANDBACK_V_REL:
      self.reset()
      return None

    v_lim = V_ENGAGE + (DISENGAGE_MARGIN if self.engaged else 0.0)
    # start braking when a constant A_ENGAGE stop is due (distance scales with speed), so faster approaches begin
    # in the 8-3 m/s band; once engaged, keep a margin so it does not flicker off
    s_engage = min(v_ego ** 2 / (2.0 * A_ENGAGE) + S_ENGAGE_BASE, S_ENGAGE_MAX) + (DISENGAGE_MARGIN if self.engaged else 0.0)
    if v_ego > v_lim or s > s_engage:
      self.reset()
      return None
    if not self.engaged:
      self.engaged = True
      self.a_prev = float(a_current)

    lead_stopped = v_lead < LEAD_STOPPED_V
    if s <= 0.0:
      decel = A_PAST  # past the point: brake firmly but not full-force, the standstill hold does the rest
    else:
      # deceleration that zeroes the closing speed at the point; it tapers to a light glide as v and s shrink,
      # like easing off the pedal for the last metre. The firm standstill hold is longcontrol + stopAccel, not here.
      decel = v_close ** 2 / (2.0 * max(s, S_MIN))
      floor = A_MIN if lead_stopped else 0.0
      decel = float(np.clip(decel, floor, -A_NEG_MAX))
    a = -decel

    a = min(a, float(a_current))                       # never brake less than the MPC
    a = min(a, self.a_prev + RELEASE_RATE * self.dt)   # and never let go with a jolt
    a = float(np.clip(a, A_NEG_MAX, 0.0))
    self.a_prev = a
    return a
