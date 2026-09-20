"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Stop gap governor: a repeatable stopping distance behind a stopped lead. The lead MPC nominally wants
STOP_DISTANCE (6 m) but its distance cost is soft, so on the road it reaches 2 m/s with only 6-7 m left and the last
metres are whatever the approach happened to leave over: 2026-09-20 rlogs show stops at 2.98, 3.42, 3.96 and 4.35 m.

Once the lead is stopped and ego is below V_ENGAGE within S_ENGAGE of the stop point, the lead MPC target is
replaced by the constant deceleration that stops ego exactly STOP_GAP metres from the lead from where it is right
now, v^2 / (2 s), re-planned every frame. There is no velocity target to chase: if the PCM brakes harder than asked
(it does at low speed on this car, 1.3-2x), the re-planned demand simply gets lighter, and with a plant gain k the
speed still goes to zero exactly at s = 0 (v^2 ~ s^k), only with a softer finish. Two earlier versions that tracked
a velocity profile released the brake to catch a faster profile and then braked hard at the end, or tapered to
nothing and let the hybrid creep off again at standstill (road tests 2026-09-20).

The demand is floored at A_MIN on the approach, at A_HOLD below V_HOLD (longcontrol holds the car at standstill with
whatever it was braking with when it stopped, so this is the parking brake), and its release is rate limited so
taking over from the MPC never lets the brake go with a jolt.
"""
import numpy as np

STOP_GAP = 3.0  # m, radar distance to the lead at the stop
V_ENGAGE = 3.0  # m/s, ego speed below which the governor takes the approach over from the MPC
S_ENGAGE = 9.0  # m, distance still to travel below which it engages
LEAD_STOPPED_V = 0.25  # m/s, lead speed below which it counts as stopped (matches the lead start assist trigger)
A_MIN = 0.2  # m/s^2, least deceleration demanded while still approaching
A_NEG_MAX = -2.0  # m/s^2, hardest braking the governor asks for (a cut-in closer than that is the MPC's problem)
A_HOLD = 0.4  # m/s^2, brake below V_HOLD and at standstill; the hybrid creeps off again with less
V_HOLD = 0.5  # m/s
S_MIN = 0.1  # m, distance used in v^2 / (2 s) at and past the stop point
RELEASE_RATE = 1.0  # m/s^3, fastest the demand may get lighter (the brake onset shaper limits the other direction)
DISENGAGE_MARGIN = 1.0  # hysteresis on V_ENGAGE and S_ENGAGE once engaged


def stopping_decel(v: float, s: float) -> float:
  """Constant deceleration (positive) that brings v to zero over s."""
  return v ** 2 / (2.0 * max(s, S_MIN))


class StopGapGovernor:
  def __init__(self, dt: float):
    self.dt = dt
    self.engaged = False
    self.a_prev = 0.0

  def reset(self) -> None:
    self.engaged = False
    self.a_prev = 0.0

  def update(self, allowed: bool, v_ego: float, lead_present: bool, d_rel: float, v_lead: float, a_current: float) -> float | None:
    """Returns the acceleration target for the approach, or None when the MPC should keep control.
    a_current is the target the MPC is asking for right now; the first governor output releases from it gradually."""
    lead_stopped = lead_present and abs(v_lead) < LEAD_STOPPED_V
    s = d_rel - STOP_GAP
    if not allowed or not lead_stopped:
      self.reset()
      return None

    v_lim = V_ENGAGE + (DISENGAGE_MARGIN if self.engaged else 0.0)
    s_lim = S_ENGAGE + (DISENGAGE_MARGIN if self.engaged else 0.0)
    if v_ego > v_lim or s > s_lim:
      self.reset()
      return None
    if not self.engaged:
      self.engaged = True
      self.a_prev = float(a_current)

    a = -float(np.clip(stopping_decel(v_ego, s), A_MIN, -A_NEG_MAX))
    if v_ego < V_HOLD or s <= 0.0:
      a = min(a, -A_HOLD)
    a = min(a, self.a_prev + RELEASE_RATE * self.dt)
    self.a_prev = a
    return a
