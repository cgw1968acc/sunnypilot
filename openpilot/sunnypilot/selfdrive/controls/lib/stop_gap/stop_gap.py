"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Stop gap governor: a repeatable stopping distance behind a stopped lead. The lead MPC nominally wants
STOP_DISTANCE (6 m) but its distance cost is soft, so on the road it reaches 2 m/s with only 6-7 m left and the last
metres are whatever the approach happened to leave over: 2026-09-20 rlogs show stops at 2.98, 3.42, 3.96 and 4.35 m.
Once the lead is stopped and ego is below V_ENGAGE, this replaces the lead MPC target with a velocity profile that
ends exactly STOP_GAP metres from the lead:

  v_max(s) = sqrt(V_TAPER^2 + 2 * a_nom * (s - S_TAPER))   for s > S_TAPER   (constant deceleration a_nom)
  v_max(s) = V_TAPER * s / S_TAPER                          for s <= S_TAPER  (deceleration fades linearly to zero)

where s is the distance still to travel. a_nom is fixed when the governor engages: the constant deceleration that
stops ego at the point from where it is right then (at least A_NOM, at most A_NOM_MAX), so engaging never steps the
brake. The profile's own deceleration is fed forward and a velocity loop corrects the rest. A small positive target
is allowed only within S_POS_MAX of the stop point while still rolling, so a stop that landed long is corrected by
rolling on, never by restarting and never on the approach.
"""
import math
import numpy as np

STOP_GAP = 3.0  # m, radar distance to the lead at the stop
V_ENGAGE = 3.0  # m/s, ego speed below which the governor takes the approach over from the MPC
S_ENGAGE = 9.0  # m, distance still to travel below which it engages
LEAD_STOPPED_V = 0.25  # m/s, lead speed below which it counts as stopped (matches the lead start assist trigger)
A_NOM = 0.8  # m/s^2, least deceleration of the constant-decel part of the profile
A_NOM_MAX = 1.5  # m/s^2, most the profile is steepened to for an ego that arrives fast; beyond that the MPC's brake stays
V_TAPER = 0.5  # m/s, speed at which the profile switches from constant deceleration to the linear taper
TAU_V = 0.5  # s, velocity loop time constant
A_POS_MAX = 0.2  # m/s^2, most the governor may ask for while rolling towards the stop point
S_POS_MAX = 2.0  # m, positive targets only this close to the stop point (a stop that landed long), never on the approach
A_NEG_MAX = -2.0  # m/s^2, hardest braking the governor asks for (a cut-in closer than that is the MPC's problem)
V_ROLLING = 0.1  # m/s, below this ego counts as stopped and only zero or negative targets are given
DISENGAGE_MARGIN = 1.0  # hysteresis on V_ENGAGE and S_ENGAGE once engaged


def taper_distance(a_nom: float) -> float:
  # the linear taper starts where its deceleration (V_TAPER^2 / S_TAPER) equals a_nom, so decel is continuous
  return V_TAPER ** 2 / a_nom


def v_max_profile(s: float, a_nom: float = A_NOM) -> float:
  if s <= 0.0:
    return 0.0
  s_taper = taper_distance(a_nom)
  if s <= s_taper:
    return V_TAPER * s / s_taper
  return math.sqrt(V_TAPER ** 2 + 2.0 * a_nom * (s - s_taper))


def a_profile(s: float, v: float, a_nom: float = A_NOM) -> float:
  """Deceleration the profile itself carries at this point: fed forward so the velocity loop only corrects errors."""
  if s <= 0.0:
    return 0.0
  s_taper = taper_distance(a_nom)
  if s <= s_taper:
    return -(V_TAPER / s_taper) * v
  return -a_nom


class StopGapGovernor:
  def __init__(self):
    self.engaged = False
    self.a_nom = A_NOM

  def reset(self) -> None:
    self.engaged = False
    self.a_nom = A_NOM

  def update(self, allowed: bool, v_ego: float, lead_present: bool, d_rel: float, v_lead: float) -> float | None:
    """Returns the acceleration target for the approach, or None when the MPC should keep control."""
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
      # steepen the profile to the constant deceleration that stops from right here, never below A_NOM
      self.a_nom = float(np.clip(v_ego ** 2 / (2.0 * max(s, 0.1)), A_NOM, A_NOM_MAX))
      self.engaged = True

    a = a_profile(s, v_ego, self.a_nom) + (v_max_profile(s, self.a_nom) - v_ego) / TAU_V
    a_pos = A_POS_MAX if (v_ego > V_ROLLING and s < S_POS_MAX) else 0.0
    return float(np.clip(a, A_NEG_MAX, a_pos))
