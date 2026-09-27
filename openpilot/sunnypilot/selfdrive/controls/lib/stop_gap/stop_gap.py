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
A_ENGAGE = 0.85  # m/s^2, start braking when a constant stop at this deceleration is due (distance scales with speed)
S_ENGAGE_BASE = 5.0  # m, added to that stopping distance (3 -> 5 on 2026-09-27 so the glide below 5 km/h fits inside)
S_ENGAGE_MAX = 55.0  # m, hard cap on how far out it engages
CREEP_LEAD_V = 2.0  # m/s, lead speed below which the governor manages the follow
LEAD_STOPPED_V = 0.25  # m/s, lead speed below which it counts as fully stopped
HANDBACK_V_REL = 0.2  # m/s, lead opening faster than ego by this much is a pull-away: hand back to the MPC
A_NOM = 0.7  # m/s^2 (0.8 -> 0.7, 2026-09-27: a touch lighter stop), least deceleration of the constant-deceleration (fast) part of the profile
A_NOM_MAX = 1.6  # m/s^2, most it steepens to for a fast/close arrival
V_TAPER = 0.8  # m/s, below this the small creep-brake floor applies
# Three-phase stop (driver, Altis 2026-09-27): a little firmer between 10 and 5 km/h, then below 5 km/h a long glide
# with only a very light brake all the way to the point.
V_FIRM = 2.8  # m/s (10 km/h): top of the firmer phase
A_FIRM_EXTRA = 0.25  # m/s^2 added to the constant-deceleration part for the 10-5 km/h phase
A_FIRM_MAX = 1.5  # m/s^2
V_GLIDE = 0.83  # m/s (3 km/h; was 5 km/h - the driver is fine with a shorter glide): below this the car glides ...
A_GLIDE = 0.2  # m/s^2 ... with this very light brake ...
A_END = 0.4  # m/s^2 ... and the last S_END metres a touch firmer so the car actually comes to rest instead of crawling
S_END = 0.5  # m
TAU_V = 0.6  # s, velocity-loop time constant that pulls ego onto the profile
A_NEG_MAX = -2.0  # m/s^2, hardest braking the governor asks for
A_PAST = 1.0  # m/s^2, firm (not full-force) brake once past the point
V_STOP_CLAMP = 0.3  # m/s, and only once nearly stopped
A_CREEP_FLOOR = 0.06  # m/s^2, smallest brake kept near the stop so the hybrid never creeps off a dead stop
S_MIN = 0.1  # m
RELEASE_RATE = 1.0  # m/s^3, fastest the demand may get lighter, from the MPC's target at engage
DISENGAGE_MARGIN = 1.0


def _phases(a_nom: float) -> tuple[float, float, float]:
  """(a_firm, s_glide, s_firm): the firmer deceleration at 10 km/h and the lengths of the glide and firm phases. The firm
  phase ramps its deceleration linearly (in distance) from a_firm at 10 km/h down to A_GLIDE at 5 km/h, so there is no
  step for the brake to release when the glide starts."""
  a_firm = min(a_nom + A_FIRM_EXTRA, A_FIRM_MAX)
  s_glide = S_END + (V_GLIDE ** 2 - 2.0 * A_END * S_END) / (2.0 * A_GLIDE)   # glide at A_GLIDE, last S_END at A_END
  s_firm = (V_FIRM ** 2 - V_GLIDE ** 2) / (a_firm + A_GLIDE)   # mean deceleration of the ramp is (a_firm + A_GLIDE) / 2
  return a_firm, s_glide, s_firm


def _profile(s: float, a_nom: float, three_phase: bool = True) -> tuple[float, float]:
  """Returns (v_target, a_feedforward>0) at distance-to-point s. Three-phase: constant a_nom while fast, a ramp from
  a_firm (10 km/h) down to A_GLIDE (5 km/h), the light glide at A_GLIDE, and the last S_END metres at A_END so the car
  comes to rest instead of crawling. Single-phase (not enough room): the original constant deceleration with the
  short linear taper."""
  if not three_phase:
    s_taper = V_TAPER ** 2 / a_nom
    if s <= 0.0:
      return 0.0, a_nom
    if s <= s_taper:
      return V_TAPER * s / s_taper, a_nom * s / s_taper   # linear v vs s -> deceleration fades to 0 at the point
    return math.sqrt(V_TAPER ** 2 + 2.0 * a_nom * (s - s_taper)), a_nom
  if s <= 0.0:
    return 0.0, A_END
  if s <= S_END:
    return math.sqrt(2.0 * A_END * s), A_END
  a_firm, s_glide, s_firm = _phases(a_nom)
  if s <= s_glide:
    return math.sqrt(2.0 * A_END * S_END + 2.0 * A_GLIDE * (s - S_END)), A_GLIDE
  if s <= s_glide + s_firm:
    x = s - s_glide
    a = A_GLIDE + (a_firm - A_GLIDE) * x / s_firm
    v2 = V_GLIDE ** 2 + 2.0 * A_GLIDE * x + (a_firm - A_GLIDE) * x * x / s_firm
    return math.sqrt(max(v2, 0.0)), a
  return math.sqrt(V_FIRM ** 2 + 2.0 * a_nom * (s - s_glide - s_firm)), a_nom


def _fits_three_phase(v_close: float, s: float) -> bool:
  """Room for the firm ramp and the glide? Otherwise (late engage, cut-in) fall back to the single-phase profile."""
  _, s_glide, s_firm = _phases(A_NOM)
  return v_close > V_FIRM and s > s_glide + s_firm + 1.0


def _a_nom_for(v_close: float, s: float) -> float:
  """The constant deceleration of the fast part that lands the three-phase profile on the point from (v_close, s)."""
  if v_close <= V_FIRM:
    return A_NOM
  a_nom = A_NOM
  for _ in range(2):  # a_firm depends on a_nom; two passes settle it
    _, s_glide, s_firm = _phases(a_nom)
    a_nom = float(np.clip((v_close ** 2 - V_FIRM ** 2) / (2.0 * max(s - s_glide - s_firm, S_MIN)), A_NOM, A_NOM_MAX))
  return a_nom


class StopGapGovernor:
  def __init__(self, dt: float):
    self.dt = dt
    self.engaged = False
    self.a_prev = 0.0
    self.a_nom = A_NOM
    self.three_phase = False

  def reset(self) -> None:
    self.engaged = False
    self.a_prev = 0.0
    self.a_nom = A_NOM
    self.three_phase = False

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
      # fix the fast-part deceleration at engage: the constant rate that, with the firm and glide phases after it,
      # lands on the point from here; at least A_NOM
      self.three_phase = _fits_three_phase(v_close, s)
      self.a_nom = _a_nom_for(v_close, s) if self.three_phase else float(np.clip(v_close ** 2 / (2.0 * max(s, S_MIN)), A_NOM, A_NOM_MAX))
      self.a_prev = float(a_current)
      self.engaged = True

    if s <= 0.0:
      a = -A_PAST
    else:
      v_prof, a_ff = _profile(s, self.a_nom, self.three_phase)
      decel = a_ff + (v_close - v_prof) / TAU_V   # feed-forward profile decel + pull onto the profile
      # keep a small brake floor near the stop (v_ego < V_TAPER) so the car never fully releases and creeps off a
      # dead stop; above that the profile can still ease to zero. The firm standstill clamp (stopAccel) does the hold.
      lo = A_CREEP_FLOOR if v_ego < V_TAPER else 0.0
      decel = float(np.clip(decel, lo, -A_NEG_MAX))
      a = -decel

    # Never brake less than the MPC - except inside the governor's own soft phases (from 10 km/h down) behind a lead
    # that is fully stopped: there the MPC (which aims 6 m short of the lead) would ask for ~0.3-1 m/s^2 and kill the
    # glide, while the closing energy is tiny (<= 10 km/h, > 3 m to go) and the standstill clamp still applies.
    _, s_glide, s_firm = _phases(self.a_nom)
    governor_owns_brake = self.three_phase and (s <= s_glide + s_firm) and (v_lead < LEAD_STOPPED_V) and (v_close <= V_FIRM + 0.3)
    if not governor_owns_brake:
      a = min(a, float(a_current))
    a = min(a, self.a_prev + RELEASE_RATE * self.dt)   # never let the brake go with a jolt
    a = float(np.clip(a, A_NEG_MAX, 0.0))
    self.a_prev = a
    # clamp for standstill once nearly stopped; the light taper above has already eased the car in near the point,
    # so this latches smoothly instead of grabbing the car short
    should_stop = v_ego < V_STOP_CLAMP
    return a, should_stop
