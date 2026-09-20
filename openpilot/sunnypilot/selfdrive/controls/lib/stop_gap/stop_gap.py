"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Low-speed lead follow with a repeatable stopping distance and a human-like stop. The lead MPC's distance cost is
soft, so behind a stopped/crawling lead it neither settles at a consistent gap nor stops smoothly. This governs the
approach to a point STOP_GAP metres behind the lead with a velocity profile that:
  - sheds speed with a firm, constant deceleration a_nom while fast (the 8-3 m/s band the driver wanted brisker),
  - then, below V_TAPER, ramps the deceleration down linearly so it fades to zero exactly at the point: the last
    metre is a light glide, and the car reaches v = 0 AT the gap rather than braking hard and stopping short.
It works in the closing frame (v_ego - vLead), so a creeping lead is followed at the same gap.

Road-test history (2026-09-20): a version that braked at a constant rate to the point stopped short of 3 m on a
faster approach and then rolled forward to close the gap (felt like "stop, move forward, brake again"); the taper
plus gating the standstill request on POSITION (only within S_STOP of the point, not merely on low speed) makes it
ease in once. It only ever adds braking (min with the MPC), hands back when the lead opens faster than ego, and the
firm standstill hold is longcontrol + the (firm) hybrid stopAccel, not this module.
"""
import math
import numpy as np

STOP_GAP = 2.4  # m, radar target; the small coast onto the clamp point settles the stop near 3.0 m
V_ENGAGE = 8.5  # m/s, ego speed below which the governor takes the stop over from the MPC
A_ENGAGE = 1.0  # m/s^2, start braking when a constant stop at this deceleration is due (distance scales with speed)
S_ENGAGE_BASE = 3.0  # m, added to that stopping distance
S_ENGAGE_MAX = 55.0  # m, hard cap on how far out it engages
CREEP_LEAD_V = 2.0  # m/s, lead speed below which the governor manages the follow
LEAD_STOPPED_V = 0.25  # m/s, lead speed below which it counts as fully stopped
HANDBACK_V_REL = 0.2  # m/s, lead opening faster than ego by this much is a pull-away: hand back to the MPC
A_NOM = 0.8  # m/s^2, least deceleration of the constant-deceleration (fast) part of the profile
A_NOM_MAX = 1.6  # m/s^2, most it steepens to for a fast/close arrival
V_TAPER = 0.8  # m/s, below this the deceleration ramps down linearly to zero at the point (the glide)
TAU_V = 0.6  # s, velocity-loop time constant that pulls ego onto the profile
A_NEG_MAX = -2.0  # m/s^2, hardest braking the governor asks for
A_PAST = 1.0  # m/s^2, firm (not full-force) brake once past the point
V_STOP_CLAMP = 0.3  # m/s, and only once nearly stopped
S_MIN = 0.1  # m
RELEASE_RATE = 1.0  # m/s^3, fastest the demand may get lighter, from the MPC's target at engage
DISENGAGE_MARGIN = 1.0


def _profile(s: float, a_nom: float) -> tuple[float, float]:
  """Returns (v_target, a_feedforward>0) at distance-to-point s for a stop with fast-part deceleration a_nom."""
  s_taper = V_TAPER ** 2 / a_nom  # where the linear taper meets the constant-deceleration part (continuous)
  if s <= 0.0:
    return 0.0, a_nom
  if s <= s_taper:
    return V_TAPER * s / s_taper, a_nom * s / s_taper   # linear v vs s -> deceleration fades to 0 at the point
  return math.sqrt(V_TAPER ** 2 + 2.0 * a_nom * (s - s_taper)), a_nom


class StopGapGovernor:
  def __init__(self, dt: float):
    self.dt = dt
    self.engaged = False
    self.a_prev = 0.0
    self.a_nom = A_NOM

  def reset(self) -> None:
    self.engaged = False
    self.a_prev = 0.0
    self.a_nom = A_NOM

  def update(self, allowed: bool, v_ego: float, lead_present: bool, d_rel: float, v_lead: float,
             a_current: float) -> tuple[float, bool] | None:
    """Returns (accel_target, should_stop), or None when the MPC should keep control."""
    s = d_rel - STOP_GAP
    lead_slow = lead_present and v_lead < CREEP_LEAD_V
    v_close = v_ego - max(v_lead, 0.0)
    if not allowed or not lead_slow or v_close < -HANDBACK_V_REL:
      self.reset()
      return None

    v_lim = V_ENGAGE + (DISENGAGE_MARGIN if self.engaged else 0.0)
    s_engage = min(v_ego ** 2 / (2.0 * A_ENGAGE) + S_ENGAGE_BASE, S_ENGAGE_MAX) + (DISENGAGE_MARGIN if self.engaged else 0.0)
    if v_ego > v_lim or s > s_engage:
      self.reset()
      return None
    if not self.engaged:
      # fix the fast-part deceleration at engage: the constant rate that stops from here, at least A_NOM
      self.a_nom = float(np.clip(v_close ** 2 / (2.0 * max(s, S_MIN)), A_NOM, A_NOM_MAX))
      self.a_prev = float(a_current)
      self.engaged = True

    if s <= 0.0:
      a = -A_PAST
    else:
      v_prof, a_ff = _profile(s, self.a_nom)
      decel = a_ff + (v_close - v_prof) / TAU_V   # feed-forward profile decel + pull onto the profile
      # clip to a brake only, but allow release to zero: if the car has fallen behind the profile (braked too hard,
      # e.g. the PCM over-delivered), the brake comes off so it coasts/creeps the last bit to the point instead of
      # stopping short and then jerking forward
      decel = float(np.clip(decel, 0.0, -A_NEG_MAX))
      a = -decel

    a = min(a, float(a_current))                       # never brake less than the MPC
    a = min(a, self.a_prev + RELEASE_RATE * self.dt)   # never let the brake go with a jolt
    a = float(np.clip(a, A_NEG_MAX, 0.0))
    self.a_prev = a
    # clamp for standstill once nearly stopped; the light taper above has already eased the car in near the point,
    # so this latches smoothly instead of grabbing the car short
    should_stop = v_ego < V_STOP_CLAMP
    return a, should_stop
