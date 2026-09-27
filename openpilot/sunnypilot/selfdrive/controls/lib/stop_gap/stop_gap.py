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
# Speed-scheduled stop (driver, Altis 2026-09-27, "firm from 15 km/h, then lighter, every change gradual"): the
# deceleration is a piecewise-linear function of speed, so it firms up and eases off progressively - no steps.
#   >= V_FIRM_IN            a_nom (constant, sized so the whole profile lands on the point)
#   V_FIRM_IN -> V_FIRM     ramps up to a_firm = a_nom + A_FIRM_EXTRA
#   V_FIRM -> V_GLIDE       ramps down to A_GLIDE
#   V_GLIDE -> V_END        A_GLIDE (the light part, a brake still on - not coasting)
#   V_END -> 0              ramps up to A_END so the car comes to rest instead of crawling
V_FIRM_IN = 5.6  # m/s (20 km/h)
V_FIRM = 4.2  # m/s (15 km/h)
A_FIRM_EXTRA = 0.4  # m/s^2 (0.5 -> 0.4, driver 2026-09-27 after the first test)
A_FIRM_MAX = 1.6  # m/s^2
V_GLIDE = 0.83  # m/s (3 km/h)
A_GLIDE = 0.3  # m/s^2
V_END = 0.7  # m/s
A_END = 0.6  # m/s^2
PROFILE_DV = 0.02  # m/s, integration step for the distance table
TAU_V = 0.6  # s, velocity-loop time constant that pulls ego onto the profile
A_NEG_MAX = -2.0  # m/s^2, hardest braking the governor asks for
A_PAST = 1.0  # m/s^2, firm (not full-force) brake once past the point
V_STOP_CLAMP = 0.3  # m/s, and only once nearly stopped
A_CREEP_FLOOR = 0.06  # m/s^2, smallest brake kept near the stop so the hybrid never creeps off a dead stop
S_MIN = 0.1  # m
RELEASE_RATE = 1.0  # m/s^3, fastest the demand may get lighter, from the MPC's target at engage
DISENGAGE_MARGIN = 1.0


def _decel_of_v(v: np.ndarray, a_nom: float) -> np.ndarray:
  a_firm = min(a_nom + A_FIRM_EXTRA, A_FIRM_MAX)
  return np.interp(v, [0.0, V_END, V_GLIDE, V_FIRM, V_FIRM_IN], [A_END, A_GLIDE, A_GLIDE, a_firm, a_nom])


class StopProfile:
  """v(s) and a(s) tables for the speed-scheduled stop: s(v) = integral of u / a(u) du from 0 to v."""
  def __init__(self, a_nom: float, v_max: float):
    self.a_nom = a_nom
    v = np.arange(0.0, max(v_max, V_FIRM_IN) + 1.0 + PROFILE_DV, PROFILE_DV)
    a = _decel_of_v(v, a_nom)
    ds = (v[1:] / a[1:]) * PROFILE_DV                       # a > 0 everywhere, so this is well defined
    self.s = np.concatenate([[0.0], np.cumsum(ds)])
    self.v = v
    self.a = a
    self.s_soft = float(np.interp(V_FIRM_IN, self.v, self.s))  # distance covered by everything below V_FIRM_IN

  def at(self, s: float) -> tuple[float, float]:
    if s <= 0.0:
      return 0.0, A_END
    if s >= self.s[-1]:  # beyond the table: constant a_nom part
      return math.sqrt(self.v[-1] ** 2 + 2.0 * self.a_nom * (s - self.s[-1])), self.a_nom
    return float(np.interp(s, self.s, self.v)), float(np.interp(s, self.s, self.a))

  def distance_to_stop(self, v: float) -> float:
    return float(np.interp(v, self.v, self.s)) if v <= self.v[-1] else self.s[-1] + (v ** 2 - self.v[-1] ** 2) / (2.0 * self.a_nom)


def _profile_single(s: float, a_nom: float) -> tuple[float, float]:
  """The original single-phase profile: constant a_nom with a short linear taper. Used when there is no room."""
  s_taper = V_TAPER ** 2 / a_nom
  if s <= 0.0:
    return 0.0, a_nom
  if s <= s_taper:
    return V_TAPER * s / s_taper, a_nom * s / s_taper
  return math.sqrt(V_TAPER ** 2 + 2.0 * a_nom * (s - s_taper)), a_nom


def _fit_profile(v_close: float, s: float) -> StopProfile | None:
  """The speed-scheduled profile whose a_nom lands on the point from (v_close, s); None when it cannot fit."""
  if v_close <= V_FIRM:
    return None
  lo, hi = A_NOM, A_NOM_MAX
  if StopProfile(hi, v_close).distance_to_stop(v_close) > s - 0.5:
    return None  # even the firmest fast part needs more room than there is
  if StopProfile(lo, v_close).distance_to_stop(v_close) <= s:
    return StopProfile(lo, v_close)   # gentlest is already enough (engaged early): use it
  for _ in range(18):  # bisection: distance falls monotonically with a_nom
    mid = 0.5 * (lo + hi)
    if StopProfile(mid, v_close).distance_to_stop(v_close) > s:
      lo = mid
    else:
      hi = mid
  return StopProfile(hi, v_close)


class StopGapGovernor:
  def __init__(self, dt: float):
    self.dt = dt
    self.engaged = False
    self.a_prev = 0.0
    self.a_nom = A_NOM
    self.profile: StopProfile | None = None

  def reset(self) -> None:
    self.engaged = False
    self.a_prev = 0.0
    self.a_nom = A_NOM
    self.profile: StopProfile | None = None

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
      self.profile = _fit_profile(v_close, s)
      self.a_nom = self.profile.a_nom if self.profile is not None else float(np.clip(v_close ** 2 / (2.0 * max(s, S_MIN)), A_NOM, A_NOM_MAX))
      self.a_prev = float(a_current)
      self.engaged = True

    if s <= 0.0:
      a = -A_PAST
    else:
      v_prof, a_ff = self.profile.at(s) if self.profile is not None else _profile_single(s, self.a_nom)
      decel = a_ff + (v_close - v_prof) / TAU_V   # feed-forward profile decel + pull onto the profile
      # keep a small brake floor near the stop (v_ego < V_TAPER) so the car never fully releases and creeps off a
      # dead stop; above that the profile can still ease to zero. The firm standstill clamp (stopAccel) does the hold.
      lo = A_CREEP_FLOOR if v_ego < V_TAPER else 0.0
      if self.profile is not None and v_ego < V_END:
        lo = max(lo, 0.75 * A_END)   # the very end always brakes: arriving slow must not turn into a crawl
      decel = float(np.clip(decel, lo, -A_NEG_MAX))
      a = -decel

    # Never brake less than the MPC - except inside the governor's own soft phases (from 10 km/h down) behind a lead
    # that is fully stopped: there the MPC (which aims 6 m short of the lead) would ask for ~0.3-1 m/s^2 and kill the
    # glide, while the closing energy is tiny (<= 10 km/h, > 3 m to go) and the standstill clamp still applies.
    governor_owns_brake = (self.profile is not None and s <= self.profile.s_soft and v_lead < LEAD_STOPPED_V
                           and v_close <= V_FIRM_IN + 0.3)
    if not governor_owns_brake:
      a = min(a, float(a_current))
    a = min(a, self.a_prev + RELEASE_RATE * self.dt)   # never let the brake go with a jolt
    a = float(np.clip(a, A_NEG_MAX, 0.0))
    self.a_prev = a
    # clamp for standstill once nearly stopped; the light taper above has already eased the car in near the point,
    # so this latches smoothly instead of grabbing the car short
    should_stop = v_ego < V_STOP_CLAMP
    return a, should_stop
