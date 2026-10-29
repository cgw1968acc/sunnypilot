# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from typing import Any

import numpy as np

# Sticky radar lead (owner 2026-10-11 night: "the radar must stick, night driving is too dangerous like this").
# Route 00000138 23:23:13-23:23:40 (C3X, Altis, 85 -> 65 km/h at night): the radar had the lead at 18-25 m the whole
# time (tracks 10196 / 10294, on the path, at our speed), but the camera lead jumped 25 -> 42 -> 31 -> 60 -> 95 m (a
# farther car) and read 6-8 m long even on the right car. Stock match_vision_to_track only takes a radar track within
# max(25 %, 5 m) of the camera distance, so every jump dropped the radar lead and leadOne became the camera's far
# target: the MPC accelerated (+0.25) toward a car it thought was 40-60 m away, then the radar matched again at 18 m.
# 17 such flips per minute of following on 138, 8.6 on 00000139, 3-7 on the 10-10 routes - a stock behaviour.
# Rule: remember the radar track that was leadOne. When the lead is no longer a radar lead (camera-only, or none), keep
# using that radar track - or the track that continues it (radar ids change) - while it stays on the model path and
# is closer than the camera's lead. A radar match is always used as it is; the kept track is only ever CLOSER than the
# camera lead, so it can only make the car brake earlier, never later. It is let go when the track leaves the path
# (the lead or we change lanes), disappears, or after STICKY_MAX_T without a radar match.
# m: the kept track must stay this close to the model path at its distance - tight up close, wider far out where the
# model path and the radar angle disagree more in a curve (138 23:23: the lead 1.1-2.2 m outside the path at 45-100 m
# in a left curve, consistently on the outside; the next lane is ~3.5 m away)
STICKY_Y_TOL_BP = [20.0, 70.0]  # m
STICKY_Y_TOL_V = [1.0, 2.0]     # m
# tncr 2026-10-22, route 00000096 09:19 (owner: "why did it stop almost 7 m behind the motorcycle and then creep only
# to ~5 m?"): a scooter waiting at the front of the motorcycle box, near the kerb, sat 1.0-1.5 m right of the model
# path (single radar frames 1.9-2.5 m) at 10-18 m. The radar had it at 10-11 m, but every frame it fell out of the
# 1.0 m band the sticky rule let it go and leadOne became the camera's 15-18 m estimate: 18 jumps 10 <-> 17 m in the
# last 5 s of the stop, so stop_gap_close restarted 16 times and never eased (the car stopped at 9.7 m). For a SLOW or
# stopped lead up close the band is wider (a two-wheeler waits at the lane edge; the next lane's cars are ~3.5 m away
# and a car there moving at our speed is not covered), and a kept track may leave the band for OFF_PATH_T before it is
# let go (single-frame radar angle spikes). The kept track is still only used when the camera lead is FARTHER.
STICKY_Y_TOL_SLOW = 1.8         # m: within STICKY_SLOW_D for a lead slower than STICKY_SLOW_V
STICKY_SLOW_D = 20.0            # m
STICKY_SLOW_V = 2.0             # m/s
OFF_PATH_T = 0.3                # s
STICKY_CONT_D = 3.0       # m: a new radar id this close to where the kept track should be continues it ...
STICKY_CONT_V = 2.0       # m/s: ... at this speed difference ...
STICKY_CONT_Y = 1.0       # m: ... and this lateral difference
STICKY_CLOSER = 2.0       # m: the camera lead must be at least this much farther for the radar track to replace it
STICKY_MAX_T = 10.0       # s without a radar match before the kept track is let go anyway
STICKY_MAX_D = 100.0      # m
RADAR_TO_CAMERA = 1.52    # m (radard.py)


class StickyRadarLead:
  def __init__(self, dt: float):
    self.dt = dt
    self.track_id: int | None = None
    self.state: tuple[float, float, float] | None = None  # dRel, yRel, vLead of the kept track
    self.t = 0.0  # s since the last radar match
    self.active = False
    self.off_t = 0.0

  def reset(self) -> None:
    self.track_id, self.state, self.t, self.active = None, None, 0.0, False
    self.off_t = 0.0
    self.why = ''

  @staticmethod
  def _on_path(c, path_x, path_y) -> bool:
    if path_x is None or len(path_x) < 2:
      return False
    y_path = -float(np.interp(c.dRel + RADAR_TO_CAMERA, path_x, path_y))
    tol = float(np.interp(c.dRel, STICKY_Y_TOL_BP, STICKY_Y_TOL_V))
    if c.dRel < STICKY_SLOW_D and abs(c.vLead) < STICKY_SLOW_V:
      tol = max(tol, STICKY_Y_TOL_SLOW)
    return abs(c.yRel - y_path) < tol

  def _find(self, tracks: dict, v_ego: float):
    if self.track_id in tracks:
      return tracks[self.track_id]
    d, y, v = self.state
    d_pred = d + (v - v_ego) * self.dt
    best = None
    for c in tracks.values():
      if abs(c.dRel - d_pred) < STICKY_CONT_D and abs(c.vLead - v) < STICKY_CONT_V and abs(c.yRel - y) < STICKY_CONT_Y:
        if best is None or abs(c.dRel - d_pred) < abs(best.dRel - d_pred):
          best = c
    return best

  def update(self, lead: dict[str, Any], v_ego: float, tracks: dict | None, path_x, path_y) -> dict[str, Any]:
    self.active = False
    self.why = ''
    if tracks is None:
      self.reset()
      return lead
    if lead.get('present', False) and lead.get('radar', False):
      tid = lead.get('radarTrackId', -1)
      c = tracks.get(tid)
      self.track_id = tid if c is not None else None
      self.state = (c.dRel, c.yRel, c.vLead) if c is not None else None
      self.t = 0.0
      self.off_t = 0.0
      return lead
    if self.state is None:
      return lead
    self.t += self.dt
    c = self._find(tracks, v_ego)
    on_path = c is not None and self._on_path(c, path_x, path_y)
    self.off_t = 0.0 if on_path else self.off_t + self.dt
    if c is None or self.t > STICKY_MAX_T or c.dRel > STICKY_MAX_D or (not on_path and self.off_t > OFF_PATH_T - 1e-6):
      why = 'lost' if c is None else ('time' if self.t > STICKY_MAX_T else ('far' if c.dRel > STICKY_MAX_D else 'path'))
      self.reset()
      self.why = why
      return lead
    self.track_id, self.state = c.identifier, (c.dRel, c.yRel, c.vLead)
    if lead.get('present', False) and lead['dRel'] < c.dRel + STICKY_CLOSER:
      return lead  # the camera lead is not farther than the radar track: nothing to fix
    self.active = True
    kept = c.get_RadarState(lead.get('modelProb', 0.0) if lead.get('present', False) else 0.0)
    return kept
