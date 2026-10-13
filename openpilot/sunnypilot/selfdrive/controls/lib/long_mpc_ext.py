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
DESIRED_DIST_TRIM_BP = [15.0 / 3.6, 40.0 / 3.6, 50.0 / 3.6, 70.0 / 3.6, 120.0 / 3.6]  # m/s
DESIRED_DIST_TRIM_V = [1.0, -5.0, -5.0, 0.0, -2.5]  # m taken OFF the desired distance (negative = added)
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

# Stop gap by personality. Owner 2026-10-13 night (route not yet uploaded): "23:24, following in Relaxed, the brake to a
# stop was very poor"; "Standard also stops too far back"; "the end of the stop in Relaxed and Standard must end at about
# the same distance as Aggressive". The MPC's desired distance carries T_FOLLOW * v at every node, so Relaxed (1.75 s) and
# Standard (1.45 s) keep 0.5 / 0.2 s more gap than Aggressive (1.25 s) all the way down the stop; the car is already
# crawling further back and the stopping controller only ever eases a stop that ends too CLOSE, so the extra stays.
# T_FOLLOW itself is not touched (owner rule). Instead, below STOP_MATCH_BP the extra (T_FOLLOW - 1.25) * v is taken off
# the desired distance per node (same x_obstacle path as the trim above): below 30 km/h every personality wants exactly the
# Aggressive distance, fading back to its own by 45 km/h. Trade-off: slow following under 30 km/h also sits at the
# Aggressive gap (Relaxed at 20 km/h: 2.8 m closer, Standard 1.1 m). Highway and cruise gaps are unchanged.
T_FOLLOW_AGGRESSIVE = 1.25  # s, = stock get_T_FOLLOW(aggressive)
STOP_MATCH_BP = [30.0 / 3.6, 45.0 / 3.6]  # m/s
STOP_MATCH_W = [1.0, 0.0]


def get_stop_match_trim(v, t_follow: float):
  """metres taken OFF the desired distance per node so that low speed matches the Aggressive distance"""
  v = np.maximum(v, 0.0)
  return max(t_follow - T_FOLLOW_AGGRESSIVE, 0.0) * v * np.interp(v, STOP_MATCH_BP, STOP_MATCH_W)


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

  def adjust(self, x_obstacles, lead_xv_0, lead_xv_1, v_plan, t_follow: float | None = None):
    x_obstacles = x_obstacles + np.column_stack([get_light_onset_margin(lead_xv_0[:, 1]), get_light_onset_margin(lead_xv_1[:, 1])])
    self.night_gap.update()
    trim = get_desired_dist_trim(v_plan) - self.night_gap.extra(v_plan)
    if t_follow is not None:
      trim = trim + get_stop_match_trim(v_plan, t_follow)
    return x_obstacles + trim[:, None]
