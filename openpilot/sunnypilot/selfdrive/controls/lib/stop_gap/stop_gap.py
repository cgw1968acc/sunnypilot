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

STOP_GAP = 3.0  # m, radar target; the small coast onto the clamp point settles the stop ~0.5 m further (2.4 landed ~3.0 m;
                # driver 2026-09-30: every stop should end 3.5 m behind the lead; +0.1 with the easing-phase lead compensation,
                # which lands ~0.1 m closer in the plant)
V_ENGAGE = 11.1  # m/s (40 km/h), ego speed below which the governor takes the stop over from the MPC. Was 8.5 (30 km/h):
                 # the driver felt the brake let go right at 30 km/h (2026-09-30), i.e. at the handover itself, where the
                 # MPC eases just before the governor engages. Taking over at 40 km/h puts the handover above the
                 # 30 km/h firm ramp and under the release limiter, so the approach is one monotonic curve from 40 km/h.
A_ENGAGE = 0.85  # m/s^2, start braking when a constant stop at this deceleration is due (distance scales with speed)
S_ENGAGE_BASE = 5.0  # m, added to that stopping distance (3 -> 5 on 2026-09-27 so the glide below 5 km/h fits inside)
S_ENGAGE_MAX = 80.0  # m, hard cap on how far out it engages (55 -> 80 with the 40 km/h engage speed: 11.1^2 / 1.7 + 5 = 77 m)
CREEP_LEAD_V = 2.0  # m/s, lead speed below which the governor manages the follow
LEAD_STOPPED_V = 0.25  # m/s, lead speed below which it counts as fully stopped
HANDBACK_V_REL = 0.2  # m/s, lead opening faster than ego by this much is a pull-away: hand back to the MPC ...
# ... but only once it has lasted HANDBACK_TIME, unless it is a clear departure (HANDBACK_V_REL_FAST). Route 000000d9 seg 12
# (2026-09-30 16:58, stopped 2.4 m behind a stopped lead): one radar sample read the lead at +0.2 m/s, the governor handed
# back for a single frame, the MPC asked +1.17, longcontrol left its stopping state for that frame and the hold was
# released - the car lurched 0.3 m/s forward and the driver had to brake.
HANDBACK_TIME = 0.3  # s
HANDBACK_V_REL_FAST = 0.8  # m/s, a lead clearly driving off hands back at once
A_NOM = 0.7  # m/s^2 (0.8 -> 0.7, 2026-09-27: a touch lighter stop), least deceleration of the constant-deceleration (fast) part of the profile
A_NOM_MAX = 2.2  # m/s^2, most it steepens to for a fast/close arrival (1.6 -> 2.2, 2026-10-01: at 1.6 a stopped lead seen late
                 # got no fitted profile, the MPC stopped hard and ~6 m short; device campaign: 3.1-3.6 m, template end)
V_TAPER = 0.8  # m/s, below this the small creep-brake floor applies
# Speed-scheduled stop (driver, Altis 2026-09-27, "firm from 15 km/h, then lighter, every change gradual"; 2026-09-30:
# "keep the 15 km/h force down to about 7 km/h, then ease off step by step, and give me the 1 -> 0 km/h values to
# tune"): the deceleration is a piecewise-linear function of speed, so it firms up and eases off progressively.
#   >= V_FIRM_IN            a_nom (constant, sized so the whole profile lands on the point)
#   V_FIRM_IN -> V_FIRM     ramps up to a_firm = a_nom + A_FIRM_EXTRA
#   V_FIRM -> V_FIRM_HOLD   holds a_firm
#   V_FIRM_HOLD -> 0        eases step by step through A_3KPH, A_2KPH, A_1KPH to A_0KPH. A_0KPH is the force the car
#                           stops on, and (since 2026-09-29) the request longcontrol freezes at until the standstill
#                           hold takes over 0.8 s (lead) / 1.1 s (no lead) later - so it also decides how much the hybrid
#                           creeps in that window (-0.3 m/s^2 crept ~0.1 m/s on 2026-09-29 08:28).
V_FIRM_IN = 8.3  # m/s (30 km/h; was 20 - driver 2026-09-27: build the firmness up slowly from 30 for a smoother feel)
V_FIRM = 4.2  # m/s (15 km/h)
V_FIRM_HOLD = 4.0 / 3.6  # m/s (4 km/h): the firm force is held from V_FIRM down to here (2026-09-30: 7 -> 4, the easing from 7 let go too much)
A_FIRM_EXTRA = 0.0  # m/s^2 (was 0.45: the 15 km/h firm hump; the driver's perfect stops are front-loaded instead, 2026-10-01)
A_FIRM_MAX = 2.0  # m/s^2 (1.6 -> 2.0, 2026-09-27 21:05: a fast approach reaches 30 km/h with ~25-30 m left, a_nom fits at
                  # 1.0-1.5 and the old cap cut the +0.45 build-up to nothing - the stop felt like one straight line)
# What "seamless" means in numbers (IMU, not the wheel-speed aEgo whose stop-instant spike is an artifact of the wheel
# sensors snapping to zero): the deceleration still present in the last 0.2 s before wheel-stop is what drops to zero
# at the stop and is felt as the nod, and the pitch rebound after it scales with it. 2026-09-27 Altis logs: driver
# stops median 0.77 m/s^2 (rebound +0.27), openpilot median 0.63 (+0.24); the gentlest openpilot endings 0.0-0.2 had
# rebounds of 0.02-0.07 and the gentlest driver endings 0.15-0.36 had 0.13-0.17. Corolla Cross driver stops end at
# 0.45 - that is why it feels easier to stop smoothly. Target: <= 0.2 m/s^2 in the last 0.2 s.
# The easing below V_FIRM_HOLD, m/s^2 at each speed (linear in between). Tune these; a_firm at 4 km/h is ~1.15 when
# a_nom is 0.7 (early engage) and up to 2.0 on a compressed fast approach. 2026-09-30 second test: 3 km/h 0.60 -> 0.80,
# 1 km/h 0.40 -> 0.38, 0 km/h 0.35 -> 0.33 (driver's values); 2 km/h set on the line between them. 2026-09-30 night, from
# the IMU analysis above: 1 km/h 0.28, 0 km/h 0.20, end floor 0.15, so the car arrives at wheel-stop with ~0.2-0.4 left
# (plant with 0.4 s lag: 0.64 -> 0.43 in the last 0.2 s, last half metre 0.7 -> 1.0 s, landing 3.61 m, no creep).
# End-of-stop law from the driver's own perfect stops (2026-10-01 01:20:03 and 01:23:20, rated with three bookmarks;
# IMU, delivered deceleration vs speed): below the knee the deceleration is proportional to speed, a = v / END_TAU,
# i.e. the speed decays exponentially and the deceleration fades to almost nothing exactly at the stop - no step for
# the body to nod on. 01:20:03: 3.0 km/h 1.19, 2.2 km/h 0.92, 1.6 km/h 0.75, 1.2 km/h 0.45, 0.9 km/h 0.33,
# 0.6 km/h 0.23, stop 0.19, rebound 0.14 (v / 0.65 s gives 1.28 / 0.94 / 0.68 / 0.51 / 0.38 / 0.26 / floor). Above the
# knee the driver brakes at a steady level (2.1-2.4 from 21 down to 9.6 km/h) - front-loaded, not a firm hump near the
# end. The brake pedal itself was held CONSTANT (760 N) through the last 0.8 s; the fade is the car's.
END_TAU = 0.65         # s
END_FLOOR = 0.52       # m/s^2 REQUEST kept to the stop: the lightest perfect stop (2026-10-01 11:43:58, pedal held at 800 N)
                       # delivered 0.26-0.38 in its last second, and below 2.5 km/h the hybrid's creep torque eats ~0.2 of
                       # a request (27 stops: request - delivered median +0.15..+0.22 at 0.4-2.5 km/h; 11:37:20 glided at
                       # 1.6 km/h on -0.35). 0.15 let the car glide and brake a second time.


def end_decel(v: float) -> float:
  """End-of-stop deceleration (positive) at speed v (m/s)."""
  return max(END_FLOOR, v / END_TAU)


A_3KPH = end_decel(3.0 / 3.6)  # 1.28, kept as names for tests and the longcontrol taper
A_2KPH = end_decel(2.0 / 3.6)  # 0.85
A_1KPH = end_decel(1.0 / 3.6)  # 0.43
A_0KPH = END_FLOOR             # 0.15
V_END = 0.7  # m/s (2.5 km/h): below this the end floor applies
A_END_FLOOR = 0.15  # m/s^2: (superseded below 2.5 km/h by the terminal band)
# Terminal band (driver 2026-10-01: "take the lightest final force of the perfect stops and hold it; the car then stops
# smoothly by itself, and small up/down adjustments of that light force control the distance"). Below V_END the
# delivered deceleration target is the kinematic need to stop on the point, kept inside a narrow light band, and the
# request adds the creep torque the car loses there.
TERM_A_LO = 0.22   # m/s^2 delivered
TERM_A_HI = 0.45   # m/s^2 delivered
CREEP_COMP = 0.20  # m/s^2 added to the request below V_END
# Re-approach (driver's four-bookmark example 2026-10-01 11:45:37: after stopping 5.7 m short he let the car creep to
# ~2.6 km/h with the brake partly released, then held ~0.35 m/s^2 to stop at 3.1 m). Only behind a fully stopped,
# steadily tracked lead, when the car has stood still for REAPPROACH_WAIT with more than REAPPROACH_MIN of room left.
REAPPROACH_MIN = 1.5   # m beyond the stop point
REAPPROACH_WAIT = 1.0  # s stopped (and the room steady) before creeping forward
REAPPROACH_V = 0.7     # m/s (2.5 km/h) most it creeps
REAPPROACH_ACCEL = 0.1   # m/s^2 request while creeping up to speed (+ ~0.2 creep torque = ~0.3, as in the example)
                    # (keep it below A_1KPH / A_0KPH so the curve above, not this floor, decides the feel)
PROFILE_DV = 0.02  # m/s, integration step for the distance table
TAU_V = 0.6  # s, velocity-loop time constant that pulls ego onto the profile
A_NEG_MAX = -2.5  # m/s^2, hardest braking the governor asks for (-2.0 -> -2.5 with A_NOM_MAX 2.2; a harder MPC request still passes)
A_PAST = 1.0  # m/s^2, firm (not full-force) brake once past the point
V_STOP_CLAMP = 0.3  # m/s, and only once nearly stopped
A_CREEP_FLOOR = 0.06  # m/s^2, smallest brake kept near the stop so the hybrid never creeps off a dead stop
S_MIN = 0.1  # m
RELEASE_RATE = 2.0  # m/s^3, fastest the demand may get lighter below V_FIRM_HOLD (1.0 -> 2.0 on 2026-09-30: at 1.0 the easing
                    # from a_firm at 4 km/h to A_0KPH could not follow the tuned curve in the ~1 s the last metre takes,
                    # so the last-metre request was set by this limiter instead of A_3KPH..A_0KPH)
# ... but while still approaching (above V_FIRM_HOLD) the brake may only get lighter very slowly. The MPC hands over at
# ~30 km/h braking harder than the governor's fitted constant part, and the velocity loop then eased the brake by up to
# 0.6 m/s^2 before the firm ramp took it back up (rlog 2026-09-27 route 000000c3: -1.69 at 30 km/h, -1.04 at 25, -1.94
# at 22; driver 2026-09-30: "between 50 and 15 km/h the brake lets go a little mid-way, not linear"). Holding the
# release to this rate keeps the approach monotonic; the small extra distance it costs is absorbed by the loop lower
# down, where releasing is by design.
RELEASE_RATE_APPROACH = 0.25  # m/s^3
DISENGAGE_MARGIN = 1.0

# Easing-phase lag compensation (driver 2026-09-30: "read each deceleration and make the last 1 -> 0 km/h silky").
# Route 000000d9 (16:58): with a -0.37 request the car delivered -0.67 to -0.78 at 0.5-1 km/h. Most of that is the
# PCM's response lag, not a gain: at crawl speed the curve changes faster than the PCM follows, so the deceleration the
# car does at each speed is the one that was requested ~0.7 km/h earlier. The feed-forward is therefore read at the
# distance the car will be at DELIVERY_LEAD later. (A learned delivered/commanded gain was tried in the plant on
# 2026-09-30 and rejected: the ratio measured during the easing phase is dominated by the lag, so on a car that does
# not over-deliver it still "learned" 1.3-1.5x, lightened the last metre, and the car crept and stopped 0.5 m short.)
DELIVERY_LEAD = 0.25  # s
# The governor only owns the brake (ignores a harder MPC request) for a comfortable stop. A profile that needs more than
# this is an urgent stop: the profile does not model the brake onset shaping and the PCM lag, so the MPC's harder
# request must be allowed through (device campaign 2026-10-01: owning a 2.7 m/s^2 profile from 30 km/h with the lead
# 17 m ahead ended in contact; letting the MPC through stopped 0.6 m short of it).
GOVERNOR_OWN_A_NOM_MAX = 2.0  # m/s^2
# In the easing phase the velocity loop may add at most this much on top of the curve. Evening drive 2026-09-30 (route
# 000000da): on compressed stops (lead 3.0-3.1 m at rest, a_firm at the 2.0 cap) the loop pushed the request to -1.08
# at 1 km/h and the car delivered -1.74 with a -1.60 jolt, while the tuned curve says 0.38. Below 3 km/h the kinetic
# energy is tiny (3 km/h needs 0.6 m at 0.6 m/s^2, 0.3 m at 1.2), so limiting the correction costs at most ~0.3 m of
# landing accuracy on a tight stop and keeps the last metre on the tuned curve in every case.
EASING_LOOP_MAX = 0.2  # m/s^2


# (2026-10-01 00:07:28 two-stage stop: the firm hump + a hard cut at 3 km/h; replaced by the exponential tail below.)


def _decel_of_v(v: np.ndarray, a_nom: float) -> np.ndarray:
  # a steady a_nom (sized so the whole stop lands on the point) down to the knee, then the driver's exponential tail
  v = np.asarray(v, dtype=float)
  return np.minimum(a_nom, np.maximum(END_FLOOR, v / END_TAU))


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
      return 0.0, A_0KPH
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
    self.pull_away_t = 0.0
    self.stopped_t = 0.0
    self.reapproach = False
    self.s_min = 1e9

  def reset(self) -> None:
    self.engaged = False
    self.a_prev = 0.0
    self.a_nom = A_NOM
    self.profile: StopProfile | None = None
    self.pull_away_t = 0.0
    self.stopped_t = 0.0
    self.reapproach = False
    self.s_min = 1e9

  def update(self, allowed: bool, v_ego: float, lead_present: bool, d_rel: float, v_lead: float,
             a_current: float) -> tuple[float, bool] | None:
    """Returns (accel_target, should_stop), or None when the MPC should keep control."""
    s = d_rel - STOP_GAP
    lead_slow = lead_present and v_lead < CREEP_LEAD_V
    v_close = v_ego - max(v_lead, 0.0)
    self.pull_away_t = self.pull_away_t + self.dt if (self.engaged and v_close < -HANDBACK_V_REL) else 0.0
    pull_away = v_close < -HANDBACK_V_REL_FAST or (v_close < -HANDBACK_V_REL and self.pull_away_t >= HANDBACK_TIME)
    if not allowed or not lead_slow or pull_away:
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

    # re-approach: stood still behind a fully stopped lead with too much room left -> creep forward and stop again lightly
    still = v_ego < 0.05 and v_lead < LEAD_STOPPED_V
    self.stopped_t = self.stopped_t + self.dt if still else 0.0
    self.s_min = min(self.s_min, s) if still else 1e9
    if (not self.reapproach and self.stopped_t >= REAPPROACH_WAIT and s > REAPPROACH_MIN and
        abs(s - self.s_min) < 0.3):
      self.reapproach = True
    if self.reapproach:
      v_cap = min(REAPPROACH_V, float(np.sqrt(2.0 * TERM_A_HI * max(s, 0.0))))
      if s <= 0.3 or v_lead >= LEAD_STOPPED_V:
        self.reapproach = False
      elif v_ego < 0.8 * v_cap and (v_ego < 0.1 or self.a_prev >= 0.0):
        self.a_prev = REAPPROACH_ACCEL
        return REAPPROACH_ACCEL, False          # creep forward (brake released) up to v_cap
      else:
        need = v_close ** 2 / (2.0 * max(s, 0.2))
        a = -(float(np.clip(need, TERM_A_LO, TERM_A_HI)) + CREEP_COMP)
        a = max(a, self.a_prev - 1.0 * self.dt)  # ease the light brake in
        self.a_prev = a
        return a, v_ego < V_STOP_CLAMP and s < 0.6

    if s <= 0.0:
      a = -A_PAST
    else:
      v_prof, a_ff = self.profile.at(s) if self.profile is not None else _profile_single(s, self.a_nom)
      loop = (v_close - v_prof) / TAU_V   # pull onto the profile
      if self.profile is not None and v_ego < V_FIRM_HOLD:
        # read the curve where the car will be once the PCM has responded, so what the car actually does in the
        # last metre is the tuned curve rather than the curve ~0.7 km/h behind it, and let the loop add only a little
        _, a_ff = self.profile.at(max(s - v_close * DELIVERY_LEAD, 0.0))
        loop = min(loop, EASING_LOOP_MAX)
      decel = a_ff + loop   # feed-forward profile decel + pull onto the profile
      # keep a small brake floor near the stop (v_ego < V_TAPER) so the car never fully releases and creeps off a
      # dead stop; above that the profile can still ease to zero. The firm standstill clamp (stopAccel) does the hold.
      lo = A_CREEP_FLOOR if v_ego < V_TAPER else 0.0
      if v_ego < V_END:
        # terminal band: the light final force, nudged up or down to stop on the point, plus the creep the car loses
        need = v_close ** 2 / (2.0 * max(s, 0.2))
        lo = max(lo, float(np.clip(need, TERM_A_LO, TERM_A_HI)) + CREEP_COMP)
      decel = float(np.clip(decel, lo, -A_NEG_MAX))
      a = -decel

    # Never brake less than the MPC - except when the governor has a fitted profile behind a FULLY stopped lead: then it
    # owns the brake for the whole episode. The profile lands on the point by construction (a_nom <= A_NOM_MAX, clip at
    # A_NEG_MAX below), and letting the MPC's harder early braking through (as before: only inside the soft part from
    # 30 km/h down) put the car ahead of the profile, so the velocity loop had to ease the brake right after the
    # handover (rlog 2026-09-27 route 000000c3: -1.69 at 30 km/h -> -1.04 at 25 -> -1.94 at 22; driver 2026-09-30:
    # "the brake lets go right at 30 km/h"). With a moving lead or no fitting profile the MPC's request still wins.
    governor_owns_brake = self.profile is not None and v_lead < LEAD_STOPPED_V and self.a_nom <= GOVERNOR_OWN_A_NOM_MAX
    a = float(np.clip(a, A_NEG_MAX, 0.0))              # the governor's own profile never asks harder than A_NEG_MAX
    if not governor_owns_brake:
      # ... but it must never brake LESS than the MPC. The clip used to sit after this min(), so a lead braking hard
      # (2026-09-27 21:05: MPC -2.8 at 35 km/h, lead 27 m closing 7 m/s) was capped to -2.0 and the stop ended 2.0 m
      # behind the lead. The MPC's harder request now passes through untouched.
      a = min(a, float(a_current))
    release = RELEASE_RATE_APPROACH if v_ego > V_FIRM_HOLD else RELEASE_RATE
    a = min(a, self.a_prev + release * self.dt)   # never let the brake go with a jolt (and barely at all while approaching)
    a = float(min(a, 0.0))
    self.a_prev = a
    # clamp for standstill once nearly stopped; the light taper above has already eased the car in near the point,
    # so this latches smoothly instead of grabbing the car short
    should_stop = v_ego < V_STOP_CLAMP
    return a, should_stop
