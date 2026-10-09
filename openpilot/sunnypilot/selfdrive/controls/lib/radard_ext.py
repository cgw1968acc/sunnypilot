"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.lead_dropout_hold import LeadDropoutHold

STATIONARY_TRACK_V = 1.0   # m/s, radar point counted as stationary
VISION_MOVING_V = 4.0      # m/s, camera lead counted as clearly moving
MATCH_MAX_Y_OFFSET = 2.5   # m, lateral radar/camera disagreement beyond which such a match is rejected
                           # (|dy| p90 0.55 m, p99 3.2 m over 22k matched frames; the misfires sat at 2.6-9 m in turns)
CORROBORATE_MAX_Y = 1.5    # m, a moving radar track this close to the camera lead's lateral position is the camera lead
MATCH_MAX_DV = 10.0        # m/s, radar/camera speed disagreement beyond which a slow radar point is not the camera lead


def reject_match(track, lead, tracks, v_ego: float, offset_vision_dist: float) -> bool:
  """tnpb2 radar-camera match filters, called after stock sanity checks."""
  def dist_sane_fn(c):
    return abs(c.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])

  # A stationary radar point well off to the side of a lead the camera sees clearly moving is a roadside object, not
  # the lead (C3X route 000000e5 2026-10-01 13:31:59: mid left turn at 20 km/h, vision lead 11 m ahead at 23 km/h,
  # radar matched a stationary point 8.9 m to the left (vision: 3.0 m) and the MPC asked -2.5 for a full second).
  # Both conditions are required: in a normal stop behind a stopped car the radar and camera agree laterally, so a
  # noisy camera speed alone never drops the radar lead.
  side_object = (v_ego + track.vRel < STATIONARY_TRACK_V and lead.v[0] > VISION_MOVING_V and
                 abs(track.yRel + lead.y[0]) > MATCH_MAX_Y_OFFSET)
  # A stationary radar point at the place of a lead the camera sees clearly moving is roadside clutter when another
  # radar track at that place is moving and agrees with the camera: the radar then corroborates the camera's moving
  # car, and a stationary point cannot be it (C3X route 000000f7 2026-10-04 14:22:12: 55 km/h in a left curve, camera
  # lead 55-65 m ahead at 21-48 km/h (jittery), radar: the lead at 18-30 km/h 0.5 m off the camera's y plus a roadside
  # point at the same range and lateral offset; the max-prob match flipped to the roadside point every few frames,
  # passing the 10 m/s gate whenever the camera speed dipped below 36 km/h, and the MPC asked -2.5 for 3 s).
  # Without a moving track at that place a stationary point is still accepted: a stopped car with a noisy camera
  # speed keeps its radar lead, and genuine stopped-vehicle detection is unchanged.
  clutter = False
  if v_ego + track.vRel < STATIONARY_TRACK_V and lead.v[0] > VISION_MOVING_V:
    clutter = any(c is not track and v_ego + c.vRel > VISION_MOVING_V and
                  abs(c.vRel + v_ego - lead.v[0]) < MATCH_MAX_DV and
                  abs(c.yRel + lead.y[0]) < CORROBORATE_MAX_Y and dist_sane_fn(c)
                  for c in tracks.values())
  return side_object or clutter


# Radar cut-in. Owner 2026-10-09: "a car coming out from the roadside must be detected; if the price is phantom braking
# I accept it, but the braking must be smooth". Route 00000119 09:19:37: a car pulled out from parked cars at
# 13-17 km/h while we drove 49 km/h. The radar had it 0.55 s before the driver braked (track at 23-25 m, 1.0 m off
# the path, moving), but the camera stayed on a farther car at 40-55 m, so stock radard (radar only as a match to the
# camera lead; radar-only below 14 km/h) did not use it until the camera saw it at 17.5 m.
# A moving radar track inside the model path, clearly closer than the current lead and slower than us, for
# CUTIN_FRAMES radard frames in a row becomes leadOne. Scan of 10 routes (claude_work/cutin_scan.py, radar tracks
# rebuilt from CAN): 29 triggers, 26 confirmed by the lead/camera within 2 s, 3 unconfirmed; 09:19:37.63 triggers 0.7 s
# before the camera. Smoothness of the resulting brake is the job of the brake onset / friction entry limits.
RADAR_TO_CAMERA = 1.52     # m (radard.py)
CUTIN_Y_TOL = 1.3          # m from the model path at the track's distance
CUTIN_D = (2.0, 40.0)      # m
CUTIN_V_MOVE = 1.5         # m/s: the track itself moves (parked cars and roadside objects never qualify)
CUTIN_VREL = -1.0          # m/s: slower than us
CUTIN_CLOSER = 3.0         # m closer than the current lead
CUTIN_V_EGO_MIN = 3.0      # m/s (below it stock's low-speed override already uses radar-only leads)
CUTIN_FRAMES = 3


class RadarCutIn:
  def __init__(self):
    self.count: dict[int, int] = {}

  def update(self, lead: dict, v_ego: float, tracks: dict, path_x, path_y) -> dict:
    if v_ego < CUTIN_V_EGO_MIN or tracks is None or path_x is None or len(path_x) < 2:
      self.count.clear()
      return lead
    lead_d = lead['dRel'] if lead.get('present') else float('inf')
    best = None
    seen = {}
    for tid, c in tracks.items():
      y_path = -float(np.interp(c.dRel + RADAR_TO_CAMERA, path_x, path_y))
      ok = (CUTIN_D[0] < c.dRel < CUTIN_D[1] and abs(c.yRel - y_path) < CUTIN_Y_TOL and v_ego + c.vRel > CUTIN_V_MOVE and
            c.vRel < CUTIN_VREL and c.dRel < lead_d - CUTIN_CLOSER)
      if ok:
        seen[tid] = self.count.get(tid, 0) + 1
        if seen[tid] >= CUTIN_FRAMES and (best is None or c.dRel < best.dRel):
          best = c
    self.count = seen
    return best.get_RadarState() if best is not None else lead


class RadardExt:
  """tnpb2 lead post-processing."""
  def __init__(self, dt: float):
    self.lead_dropout_hold = LeadDropoutHold(dt)
    self.cut_in = RadarCutIn()

  def lead_one(self, lead: dict, v_ego: float, tracks: dict | None = None, path=None) -> dict:
    if path is not None:
      lead = self.cut_in.update(lead, v_ego, tracks, np.array(path.x), np.array(path.y))
    return self.lead_dropout_hold.update(lead, v_ego)
