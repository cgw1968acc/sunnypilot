"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
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


class RadardExt:
  """tnpb2 lead post-processing."""
  def __init__(self, dt: float):
    self.lead_dropout_hold = LeadDropoutHold(dt)

  def lead_one(self, lead: dict, v_ego: float) -> dict:
    return self.lead_dropout_hold.update(lead, v_ego)
