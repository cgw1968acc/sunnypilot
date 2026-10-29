# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.sticky_radar_lead import StickyRadarLead, STICKY_MAX_T, OFF_PATH_T

DT = 0.05
PATH_X = np.linspace(0.0, 150.0, 33)
PATH_Y = np.zeros(33)


class T:
  def __init__(self, identifier, d, y=0.0, v=18.0, v_ego=20.0):
    self.identifier, self.dRel, self.yRel, self.vLead = identifier, d, y, v
    self.vRel, self.vLeadK, self.aLeadK, self.aLeadTau = v - v_ego, v, 0.0, 1.5

  def get_RadarState(self, model_prob=0.0):
    return {'dRel': self.dRel, 'yRel': self.yRel, 'vRel': self.vRel, 'vLead': self.vLead, 'vLeadK': self.vLeadK,
            'aLeadK': self.aLeadK, 'aLeadTau': self.aLeadTau, 'present': True, 'modelProb': model_prob, 'radar': True,
            'radarTrackId': self.identifier}


def vision(d):
  return {'present': True, 'radar': False, 'dRel': d, 'vLead': 20.0, 'modelProb': 0.9, 'radarTrackId': -1}


def test_keeps_the_radar_lead_when_the_camera_jumps_to_a_farther_car():
  # route 00000138 23:23: radar lead at 20 m, the camera lead jumps to 45 m
  s = StickyRadarLead(DT)
  t = T(10196, 20.0)
  s.update(t.get_RadarState(), 20.0, {10196: t}, PATH_X, PATH_Y)
  out = s.update(vision(45.0), 20.0, {10196: t}, PATH_X, PATH_Y)
  assert out['radar'] and out['dRel'] == 20.0 and s.active


def test_follows_the_continuing_track_when_the_radar_id_changes():
  s = StickyRadarLead(DT)
  a = T(10196, 20.0)
  s.update(a.get_RadarState(), 20.0, {10196: a}, PATH_X, PATH_Y)
  b = T(10294, 19.9, y=0.2)
  out = s.update(vision(45.0), 20.0, {10294: b}, PATH_X, PATH_Y)
  assert out['radarTrackId'] == 10294


def test_never_replaces_a_closer_camera_lead_or_a_radar_match():
  s = StickyRadarLead(DT)
  t = T(1, 30.0)
  s.update(t.get_RadarState(), 20.0, {1: t}, PATH_X, PATH_Y)
  assert s.update(vision(29.0), 20.0, {1: t}, PATH_X, PATH_Y)['radar'] is False   # camera lead closer: kept as is
  other = T(2, 40.0)
  assert s.update(other.get_RadarState(), 20.0, {1: t, 2: other}, PATH_X, PATH_Y)['radarTrackId'] == 2


def test_lets_go_when_the_track_leaves_the_path_or_disappears():
  s = StickyRadarLead(DT)
  t = T(1, 20.0)
  s.update(t.get_RadarState(), 20.0, {1: t}, PATH_X, PATH_Y)
  t.yRel = 2.5  # the lead changed lanes: let go once it has been off the path for OFF_PATH_T
  outs = [s.update(vision(45.0), 20.0, {1: t}, PATH_X, PATH_Y)['radar'] for _ in range(int(OFF_PATH_T / DT) + 2)]
  assert outs[-1] is False
  t2 = T(1, 20.0)
  s.update(t2.get_RadarState(), 20.0, {1: t2}, PATH_X, PATH_Y)
  assert s.update(vision(45.0), 20.0, {}, PATH_X, PATH_Y)['radar'] is False


def test_far_track_in_a_curve_gets_a_wider_path_tolerance():
  s = StickyRadarLead(DT)
  t = T(1, 70.0, y=-1.8)  # 1.8 m outside the path at 70 m (route 00000138, left curve)
  s.update(t.get_RadarState(), 20.0, {1: t}, PATH_X, PATH_Y)
  assert s.update(vision(95.0), 20.0, {1: t}, PATH_X, PATH_Y)['radar']
  near = T(2, 20.0, y=1.5)  # but 1.5 m off at 20 m is not on the path
  s2 = StickyRadarLead(DT)
  s2.update(near.get_RadarState(), 20.0, {2: near}, PATH_X, PATH_Y)
  outs = [s2.update(vision(45.0), 20.0, {2: near}, PATH_X, PATH_Y)['radar'] for _ in range(int(OFF_PATH_T / DT) + 2)]
  assert outs[-1] is False


def test_slow_lead_at_the_lane_edge_is_kept():
  # route 00000096 09:19: a scooter waiting near the kerb, 1.0-1.5 m right of the path at 10-18 m, single frames 2.5 m
  s = StickyRadarLead(DT)
  t = T(1, 14.0, y=-1.0, v=0.0, v_ego=2.0)
  s.update(t.get_RadarState(), 2.0, {1: t}, PATH_X, PATH_Y)
  t.yRel = -1.6
  assert s.update(vision(17.0), 2.0, {1: t}, PATH_X, PATH_Y)['radar']
  t.yRel = -2.5  # one radar angle spike
  assert s.update(vision(17.0), 2.0, {1: t}, PATH_X, PATH_Y)['radar']
  t.yRel = -1.3
  assert s.update(vision(17.0), 2.0, {1: t}, PATH_X, PATH_Y)['dRel'] == 14.0
  # a car in the next lane moving at our speed keeps the narrow band
  s2 = StickyRadarLead(DT)
  c = T(2, 14.0, y=-1.6, v=18.0)
  s2.update(c.get_RadarState(), 20.0, {2: c}, PATH_X, PATH_Y)
  outs = [s2.update(vision(30.0), 20.0, {2: c}, PATH_X, PATH_Y)['radar'] for _ in range(int(OFF_PATH_T / DT) + 2)]
  assert outs[-1] is False


def test_gives_up_after_the_max_time_without_a_radar_match():
  s = StickyRadarLead(DT)
  t = T(1, 20.0, v=20.0)
  s.update(t.get_RadarState(), 20.0, {1: t}, PATH_X, PATH_Y)
  out = None
  for _ in range(int(STICKY_MAX_T / DT) + 2):
    out = s.update(vision(45.0), 20.0, {1: t}, PATH_X, PATH_Y)
  assert out['radar'] is False
