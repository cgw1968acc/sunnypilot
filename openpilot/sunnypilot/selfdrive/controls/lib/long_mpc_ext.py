"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import platform

import numpy as np
from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.night_gap import NightGap


# SP (Altis, driver 2026-10-03): the MPC wants v^2/(2*COMFORT_BRAKE) + T_FOLLOW*v + STOP_DISTANCE in front of it at
# every horizon node. Both constants are compiled into the prebuilt solver, so the driver's adjustment is applied from
# outside through x_obstacle, a per-node parameter: a positive trim moves the obstacle away = that much LESS desired
# distance. Driver's spec: "below 50 km/h one metre less than the original; take 70 km/h as the turning point: below
# it gradually less, above it slowly more": -1 m up to 50 km/h, back to stock at 70, then +0.05 m per km/h (+1 m at
# 90, +2 m at 110, +2.5 m at 120). The same trim applies at equal speeds, so the steady following gap moves by the
# same amount (T_FOLLOW*v + 6 - trim).
# On top of the trim, the distance the light first touch of the brake needs (driver 2026-10-03 03:45: "at 40 km/h I
# want: start a little later (the -1 m) PLUS the distance the light initial press needs" - not the +15 m a 1.5 m/s^2
# comfort brake gave at 40): LIGHT_ONSET_T_V seconds of travel at the current speed, full up to 50 km/h, fading to zero
# at 80 km/h so highway braking is untouched. The lead's stopped-equivalence gets the same term, so at equal speeds it
# cancels and the steady following gap is set by the trim alone; for a stopped lead the whole term counts. Net onset
# shift for a stopped lead: +1.8 m at 20 km/h, +3.2 at 30, +4.6 at 40, +5.9 at 50, +0.5 at 80 (trim only).
# Driver 2026-10-06 (route 00000100 19:41: close following at 44 km/h, the lead braked at -3.3, openpilot peaked at
# -3.5 / the car at -4.3): "close following at 40 km/h may really need earlier braking". An MPC replay of 19:41 on the
# C3X: a constant extra gap does not lower the peak (the stop just ends further back), but extra gap at 40 km/h that
# returns to the old value by 15 km/h is used up during the stop: +3 m -> peak -3.44 (was -3.60), +5 m -> -3.25, same
# 5.0 m final gap (the trim is read per node at the planned speed). The driver chose the +3 m version: -1 m (as before)
# up to 15 km/h, +2 m from 40 to 50 km/h (3 m more than before), stock at 70, the highway part unchanged.
# Owner 2026-10-09 (route 00000119 09:23, Aggressive at 46 km/h 23 m behind, the lead braked to a stop, request
# -2.62): "close following at 40 km/h should keep a bit more distance, so there is time to brake softly" -> +6 m from
# 40 to 50 km/h (4 m more). C3X closed-loop replay of 09:23 started at 09:22:20 so the gap settles first
# (claude_work/mpc_replay_gap_dev.py): gap before the event 23.4 -> 27.2 m, peak -2.85 -> -2.54, time below -1.5
# 3.6 -> 3.5 s, final stop gap unchanged 4.5 m (+2 m: -2.68, +6 m: -2.42, +8 m: -2.31). The plan's onset rate stays
# ~-4 m/s^3; the onset feel is the friction-entry limiter's job. Applies to every personality.
# Owner 2026-10-10: "+4 m -> +3 m, let's try" -> +5 m from 40 to 50 km/h (3 m more than before 10-09): gap before the
# 09:23 event about 26.3 m, peak about -2.61 (between the replayed +2 m and +4 m rows).
# tncr, owner 2026-10-15 evening (after route 00000073 17:16:08, close follow at 34 km/h, the lead braked hard and the
# car ended 2.2 m out): 'keep +3 m at 50-40 km/h (the extension the owner knows - in this table it is the +5 m row,
# 3 m more than before 10-09), but only +1 m at 10 km/h, interpolated in between'. Was: back to -1 m already at
# 15 km/h, i.e. +3.6 m at 34 km/h, 0 at 19, -1 at 15. Now +1 m at 10 km/h, +1.7 at 15, +2.3 at 20, +3.7 at 30,
# +4.2 at 34, +5 at 40-50; below 5 km/h the old -1 m stays so the end of a stop (stop-gap layers) is unchanged.
# Owner 2026-10-15 late evening, option 1 after the closed-loop sim (claude_work/tncr_1015pm/sim): with +1 m at 10 and
# +2.3 m at 20 km/h a lead that brakes slowly from 29 km/h (route 6b 09:19:54, a 3-bookmark stop at 4.6 m) made the
# MPC stop 6.5 m out (OLD table 4.4 m), and the gap-close layer cannot creep that in on the Cross. The hard-lead stops
# (17:16 3.3 m, 17:20 3.3 m) did not need the low-speed part. -> the low end is scaled back: 0 at 10 km/h, +1 at 20,
# +3.7 at 30 (unchanged by interpolation), +5 at 40-50 as before; 5 km/h -1 m and the highway part unchanged.
DESIRED_DIST_TRIM_BP = [5.0 / 3.6, 10.0 / 3.6, 20.0 / 3.6, 30.0 / 3.6, 40.0 / 3.6, 50.0 / 3.6, 70.0 / 3.6, 120.0 / 3.6]  # m/s
# Owner 2026-10-15 night (road tests of ebff239099 / 8e08ba7e31: "brakes too early and stops too far - a close-follow
# add must not apply to every situation; following traffic changes every moment, a static distance estimated once and
# never adjusted cannot be right"): the close-follow part of this table is now DYNAMIC (DYN_* below) and the static
# table keeps only the old -1 m at 5 km/h and the highway part (0 at 70, +2.5 m at 120).
DESIRED_DIST_TRIM_BP = [5.0 / 3.6, 10.0 / 3.6, 70.0 / 3.6, 120.0 / 3.6]  # m/s
DESIRED_DIST_TRIM_V = [1.0, 0.0, 0.0, -2.5]  # m taken OFF the desired distance (negative = added)

# Dynamic close-follow gap (tncr, owner 2026-10-15 night). The extra room the owner wants when the lead brakes hard
# ("+3 m at 50-40 km/h so the brake can start light") is added ONLY while the lead is braking, scaled by how hard:
# nothing while the lead decelerates lighter than DYN_A0 (ordinary slow-downs, a coasting lead, a stopped lead ahead),
# full DYN_GAP_V (by the planned ego speed per node, 0 below 20 km/h) from DYN_A1. It comes in fast (DYN_RATE_ON, so the
# onset moves earlier the moment the radar sees the braking) and goes away slowly (DYN_RATE_OFF). It is taken away as
# soon as the lead is nearly stopped (below DYN_V_LEAD_OFF: the stop point is known, the car still has 15-20 km/h to
# settle to the normal 3-3.5 m without creeping), when the lead pulls away, or when we stop. Why dynamic: the static
# +2..+5 m made every stop start early and the slow-lead stops end 5-6 m out (sim 6b 09:19:54: 6.07 m), and the Cross
# cannot creep that in.
DYN_GAP_BP = [20.0 / 3.6, 30.0 / 3.6, 40.0 / 3.6, 50.0 / 3.6, 70.0 / 3.6]  # m/s planned ego speed per node
DYN_GAP_V = [0.0, 2.0, 5.0, 5.0, 0.0]  # m ADDED at full urgency
DYN_A0 = -0.8        # m/s^2 lead decel: no add while the lead brakes lighter than this
DYN_A1 = -2.0        # m/s^2: full add from here
DYN_RATE_ON = 4.0    # 1/s: 0.25 s to full
DYN_RATE_OFF = 0.67  # 1/s: 1.5 s to zero
DYN_V_LEAD_OFF = 10.0 / 3.6  # m/s: the lead is nearly stopped -> take the add away
DYN_DT = 0.05        # s: the planner runs at 20 Hz


# Driver 2026-10-04 night: "bring the soft brake at 40-45 km/h a little earlier, i.e. slightly more distance to the
# lead at that speed" -> the reserved time is speed scheduled: 0.8 s up to 35 km/h, 1.1 s from 40 to 45, back to 0.8
# at 50, fading to 0 at 80 as before. Margin before the -1 m trim: +7.8 m at 35 km/h, +12.2 at 40, +13.8 at 45,
# +11.1 at 50 (was +7.9 / +8.9 / +10.0 / +11.1).
# Driver 2026-10-05: "every brake from an extremely light touch; take the distance earlier if needed" -> 1.1 s held
# from 40 to 55 km/h (was back to 0.8 at 50), then fading to 0 at 80 as before.
LIGHT_ONSET_BP = [35.0 / 3.6, 40.0 / 3.6, 55.0 / 3.6, 80.0 / 3.6]  # m/s
LIGHT_ONSET_T_V = [0.8, 1.1, 1.1, 0.0]  # s of travel reserved for the light first press


# Relaxed jerk factor. Owner 2026-10-09 (route 00000119 09:11:27, Relaxed at 61 km/h: a lead moving into the path was
# confirmed only at 54 m, the plan went 0 -> -1.9 in 1.2 s): "make the first braking smoother, but it must still stop
# smoothly". Stock uses 1.0 for relaxed (0.5 aggressive); a higher factor weights the jerk and accel-change costs more,
# so the plan eases in. Closed-loop MPC replays on the C3X (claude_work/mpc_replay119_dev.py):
#   09:11:27        max plan jerk -4.8 -> -1.8 m/s^3, peak -2.02 -> -1.88, closest gap 16.8 -> 15.0 m
#   117 23:09:50    peak -3.04 -> -2.97, jerk -6.3 -> -5.3, closest gap 29.5 -> 28.9 m
#   three Relaxed stops (09:07:56 / 09:10:14 / 09:16:35): below 15 km/h unchanged, decel at 5 km/h -0.50 -> -0.55,
#   final gap 0.5-0.8 m shorter (e.g. 4.5 -> 3.7 m)
# Standard and aggressive keep the stock factor.
RELAXED_JERK_FACTOR = 3.0

# Emergency stop-distance trim (tncr, owner 2026-10-15, supreme braking principle): "when the lead brakes hard, do not
# insist on stopping 3 m or more back - allow down to 2 m, so the brake is not too fierce". STOP_DISTANCE (6 m) is
# compiled into the solver, so - like the trim above - the obstacle is moved away while the lead brakes (<= EMERG_A) AND
# our own plan is already fierce: the trim follows the firmest planned decel seen in this event, 0 at EMERG_PLAN_A0 up to
# EMERG_TRIM at EMERG_PLAN_A1 (latched, so a softer plan does not take it away again). Ordinary stops (plan lighter than
# EMERG_PLAN_A0) are untouched, so they still end 3-3.5 m. It ramps in at EMERG_RATE_ON and out at EMERG_RATE_OFF; it
# ends when the car stops, the lead pulls away, or the lead has been calm (> EMERG_CALM_A, not closing) for EMERG_CALM_T.
# Device closed-loop MPC replays (C4, see the commit): route 00000119 09:23 (lead -2.2..-2.4 from 46 km/h) the peak
# falls -2.61 -> -2.41 (0.45 s car lag: -2.83 -> -2.55), closest 4.24 -> 2.40 m (2.30); route 00000157 23:29:35 (lead
# -3.6 from 55 km/h) stays at the -3.5 limit (physics), only the stop ends closer: 4.79 -> 2.41 m (2.90).
EMERG_A = -1.5           # m/s^2: the lead brakes at least this hard
EMERG_PLAN_A0 = -2.0     # m/s^2: no trim while the plan is lighter than this
EMERG_PLAN_A1 = -2.5     # m/s^2: full trim from here
EMERG_TRIM = 2.5         # m taken off the desired distance at full trim
EMERG_RATE_ON = 4.0      # m/s
EMERG_RATE_OFF = 1.0     # m/s
EMERG_CALM_A = -0.5      # m/s^2
EMERG_CALM_T = 2.0       # s
EMERG_DT = 0.05          # s: the planner runs at 20 Hz


# Soft brake onset inside the MPC (tncr, owner 2026-10-15 night): "the point of the extra distance at 40-50 km/h is to
# enter a SOFT brake earlier, so that when the lead suddenly brakes hard there is time to raise the force slowly and
# smoothly - not to stop further back and creep". Giving the MPC more room only moves its onset earlier; it still
# plans the same steep rise (route 77 21:28:52: a stopped lead 58 m ahead at 44 km/h, the plan went -0.7 -> -2.1 in
# 1.5 s, then coasted at -0.4 for 20 m). So the plan's brake limit (the MPC's a_min parameter, per horizon node) is now
# scheduled from the start of a brake event: CAP_A0 at the start, growing CAP_RATE per second (CAP_RATE_URGENT at full
# lead urgency, the DYN_* factor), and never below CAP_NEED_MARGIN times the constant decel that would stop CAP_STOP_D
# short of the REAL obstacle (lead + its own stopping distance, before any margin - the margins exist to move the onset,
# not the stop), so a light cap can never make the stop late or hard. Each node further down the horizon
# gets the cap of its own time, so the MPC plans the slow build itself and its internal state stays consistent.
# Lead stops only (no cap without a lead), full between 25 and 55 km/h, none below 15 (the end rules own it) or
# above 70 km/h (highway untouched). A brake event starts when the plan asks less than CAP_BRAKING_A and ends after
# CAP_RELEASE_T without braking. Hook: the one line in LongitudinalMpc.update that sets params[:, 0].
CAP_V_BP = [15.0 / 3.6, 25.0 / 3.6, 55.0 / 3.6, 70.0 / 3.6]  # m/s
CAP_V_W = [0.0, 1.0, 1.0, 0.0]
CAP_A0 = 0.6            # m/s^2: the brake the plan may ask at the start of a brake event
CAP_RATE = 0.4          # m/s^3: the cap grows this fast with a calm lead (-1.0 at 1 s, -1.4 at 2 s, -2.0 at 3.5 s)
CAP_RATE_URGENT = 2.0   # m/s^3: at full lead urgency (lead braking -2.0 or harder)
CAP_NEED_MARGIN = 1.15  # the cap never sits below this times the constant decel needed to stop CAP_STOP_D short
CAP_STOP_D = 3.5        # m short of the obstacle (a stopped lead: ~3-3.5 m behind it)
CAP_BRAKING_A = -0.2    # m/s^2: the plan counts as braking below this
CAP_RELEASE_T = 1.0     # s without braking -> the brake event ends
CAP_DT = 0.05           # s: the planner runs at 20 Hz
ACCEL_MIN_MPC = -3.5    # m/s^2: the stock limit (opendbc ACCEL_MIN), used when no cap applies


# Relaxed (far) highway gap (tncr, owner 2026-10-17 night): "for far following add following distance from 60 km/h up
# to 130 km/h, +20 m at 130 km/h, interpolated below" - only with the distance button at relaxed (owner's choice). On
# top of everything else (the trim table, the night +5 m). T_FOLLOW stays untouched (owner rule): the metres go through
# the same per-node obstacle shift as the night gap, at the planned speed of each node. Steady gap at 130 km/h in
# relaxed ~69 -> ~89 m (+0.55 s), at 95 km/h +10 m.
FAR_GAP_BP = [60.0 / 3.6, 130.0 / 3.6]  # m/s
FAR_GAP_V = [0.0, 20.0]                 # m added to the desired distance


def get_far_gap_extra(v_plan, personality) -> np.ndarray:
  """metres to ADD to the desired distance per node (relaxed only)"""
  if personality != log.LongitudinalPersonality.relaxed:
    return np.zeros_like(np.asarray(v_plan, dtype=float))
  return np.interp(np.maximum(v_plan, 0.0), FAR_GAP_BP, FAR_GAP_V)


def get_light_onset_margin(v):
  v = np.maximum(v, 0.0)
  return v * np.interp(v, LIGHT_ONSET_BP, LIGHT_ONSET_T_V)


def get_desired_dist_trim(v_ego):
  """obstacle shift per node: the driver's trim minus the light-onset margin"""
  v = np.maximum(v_ego, 0.0)
  return np.interp(v, DESIRED_DIST_TRIM_BP, DESIRED_DIST_TRIM_V) - get_light_onset_margin(v)


class LongMpcObstacleExt:
  """tnpb2 desired-distance change, applied by stock LongitudinalMpc.update()
  through one hook right after it stacks the lead obstacles: the light-onset margin is added to each lead's
  stopped-equivalence obstacle (by that lead's predicted speed) and the trim minus the margin is added per node at the
  planned ego speed. The margins are added before the stock code picks the closest lead, exactly as before."""
  @staticmethod
  def jerk_factor(personality, stock_factor: float) -> float:
    """called by stock LongitudinalMpc.set_weights: the relaxed jerk factor (RELAXED_JERK_FACTOR)"""
    return RELAXED_JERK_FACTOR if personality == log.LongitudinalPersonality.relaxed else stock_factor

  def __init__(self):
    mem_params = None
    try:
      from openpilot.common.params import Params
      mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else None
    except Exception:
      pass
    self.night_gap = NightGap(mem_params)  # +5 m from 80 km/h at night, see night_gap.py
    self.emerg = False
    self.emerg_trim = 0.0
    self.emerg_plan = 0.0  # firmest planned decel of this event
    self.calm_t = 0.0
    self.dyn = 0.0  # dynamic close-follow gap factor 0..1, see DYN_*
    self.cap_t_on = 0.0   # s since this brake event started, see CAP_*
    self.cap_t_off = 0.0  # s since the plan stopped braking
    self.a_min = None     # per-node brake limit for the MPC, None = stock

  def update_onset_cap(self, x_obstacle_0: float, v_plan, t1: float, t_idxs, closing: float, dyn: float):
    """tncr: the per-node brake limit (negative, m/s^2) for this planner step, see CAP_*"""
    v_ego = float(v_plan[0])
    a0 = (float(v_plan[1]) - v_ego) / t1
    if a0 < CAP_BRAKING_A:
      self.cap_t_on += CAP_DT
      self.cap_t_off = 0.0
    else:
      self.cap_t_off += CAP_DT
      if self.cap_t_off >= CAP_RELEASE_T:
        self.cap_t_on = 0.0
    w = float(np.interp(v_ego, CAP_V_BP, CAP_V_W))
    if w <= 0.0 or closing < -1.0 or self.cap_t_on <= 0.0:
      self.a_min = None
      return None
    rate = CAP_RATE + (CAP_RATE_URGENT - CAP_RATE) * dyn
    need = v_ego ** 2 / (2.0 * max(x_obstacle_0 - CAP_STOP_D, 0.5))
    cap = CAP_A0 + rate * (self.cap_t_on + np.asarray(t_idxs))
    cap = np.minimum(np.maximum(cap, CAP_NEED_MARGIN * need), -ACCEL_MIN_MPC)
    self.a_min = -(w * cap + (1.0 - w) * -ACCEL_MIN_MPC)
    return self.a_min

  def min_accel(self, accel_min: float):
    """called by stock LongitudinalMpc.update for params[:, 0]: the onset cap when one applies, else accel_min"""
    return self.a_min if self.a_min is not None else accel_min

  def update_dynamic_gap(self, lead_xv_0, v_plan, t1: float) -> float:
    """tncr: the dynamic close-follow gap factor (0..1) for this planner step, see DYN_*"""
    v_ego = float(v_plan[0])
    v_lead = float(lead_xv_0[0, 1])
    a_lead = (float(lead_xv_0[1, 1]) - v_lead) / t1
    closing = v_ego - v_lead
    if v_ego < 0.3 or closing < -1.0 or v_lead < DYN_V_LEAD_OFF:
      target = 0.0
    else:
      target = float(np.interp(a_lead, [DYN_A1, DYN_A0], [1.0, 0.0]))
    self.dyn = float(np.clip(target, self.dyn - DYN_RATE_OFF * DYN_DT, self.dyn + DYN_RATE_ON * DYN_DT))
    return self.dyn

  def update_emergency(self, lead_xv_0, v_plan, t1: float, t_idxs=None) -> float:
    """tncr: the emergency trim (m) for this planner step, see EMERG_*"""
    v_ego = float(v_plan[0])
    v_lead = float(lead_xv_0[0, 1])
    a_lead = (float(lead_xv_0[1, 1]) - v_lead) / t1
    closing = v_ego - v_lead
    a_plan = float(np.min(np.diff(v_plan) / np.diff(t_idxs))) if t_idxs is not None else 0.0
    self.calm_t = self.calm_t + EMERG_DT if (a_lead > EMERG_CALM_A and closing < 1.0) else 0.0
    if v_ego < 0.3 or closing < -1.0 or self.calm_t >= EMERG_CALM_T:
      self.emerg, self.emerg_plan = False, 0.0
    elif a_lead <= EMERG_A and a_plan <= EMERG_PLAN_A0:
      self.emerg = True
    if self.emerg:
      self.emerg_plan = min(self.emerg_plan, a_plan)
    target = EMERG_TRIM * float(np.interp(self.emerg_plan, [EMERG_PLAN_A1, EMERG_PLAN_A0], [1.0, 0.0])) if self.emerg else 0.0
    self.emerg_trim = float(np.clip(target, self.emerg_trim - EMERG_RATE_OFF * EMERG_DT, self.emerg_trim + EMERG_RATE_ON * EMERG_DT))
    return self.emerg_trim

  def adjust(self, x_obstacles, lead_xv_0, lead_xv_1, v_plan, t1: float = 0.0, t_idxs=None, personality=None):
    x_obstacle_raw = float(np.min(x_obstacles[0]))  # the real obstacle (lead + its stopping distance), before any margin
    x_obstacles = x_obstacles + np.column_stack([get_light_onset_margin(lead_xv_0[:, 1]), get_light_onset_margin(lead_xv_1[:, 1])])
    self.night_gap.update()
    emerg = self.update_emergency(lead_xv_0, v_plan, t1, t_idxs) if t1 > 0.0 else 0.0
    dyn = self.update_dynamic_gap(lead_xv_0, v_plan, t1) if t1 > 0.0 else 0.0
    dyn_gap = dyn * np.interp(np.maximum(v_plan, 0.0), DYN_GAP_BP, DYN_GAP_V)
    far = get_far_gap_extra(v_plan, personality)
    x_obstacles = x_obstacles + (get_desired_dist_trim(v_plan) - self.night_gap.extra(v_plan) - far + emerg - dyn_gap)[:, None]
    if t1 > 0.0 and t_idxs is not None:
      self.update_onset_cap(x_obstacle_raw, v_plan, t1, t_idxs, float(v_plan[0]) - float(lead_xv_0[0, 1]), dyn)
    return x_obstacles
