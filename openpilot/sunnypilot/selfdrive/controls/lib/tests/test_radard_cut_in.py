from types import SimpleNamespace as NS

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.radard_ext import RadarCutIn, CUTIN_FRAMES

PATH_X = np.linspace(0.0, 100.0, 33)
PATH_Y = np.zeros(33)
FAR_LEAD = {'present': True, 'dRel': 45.0, 'radarTrackId': -1}


def track(tid, d, y, v_abs, v_ego):
  return NS(identifier=tid, dRel=d, yRel=y, vRel=v_abs - v_ego, vLead=v_abs, vLeadK=v_abs, aLeadK=0.0,
            aLeadTau=NS(x=1.5), get_RadarState=lambda tid=tid, d=d: {'present': True, 'dRel': d, 'radarTrackId': tid, 'radar': True})


def run(tracks_per_frame, v_ego=13.6, lead=FAR_LEAD):
  c = RadarCutIn()
  return [c.update(lead, v_ego, t, PATH_X, PATH_Y) for t in tracks_per_frame]


def test_car_pulling_out_into_the_path_becomes_the_lead():
  # route 00000119 09:19:37: 13 km/h car 24 m ahead, 1.0 m off the path, we drive 49 km/h, camera lead at 45 m
  frames = [{122: track(122, 24.0 - 0.5 * i, -1.0, 3.7, 13.6)} for i in range(5)]
  out = run(frames)
  assert all(o['dRel'] == 45.0 for o in out[:CUTIN_FRAMES - 1])
  assert out[CUTIN_FRAMES - 1]['radarTrackId'] == 122


def test_parked_cars_never_qualify():
  frames = [{120: track(120, 25.0 - 0.7 * i, -1.0, 0.0, 13.6)} for i in range(6)]
  assert all(o is FAR_LEAD for o in run(frames))


def test_off_path_traffic_is_ignored():
  frames = [{7: track(7, 20.0, -3.4, 5.0, 13.6)} for _ in range(6)]
  assert all(o is FAR_LEAD for o in run(frames))


def test_not_closer_than_the_current_lead():
  lead = {'present': True, 'dRel': 22.0, 'radarTrackId': 5}
  frames = [{9: track(9, 21.0, 0.0, 5.0, 13.6)} for _ in range(6)]
  assert all(o is lead for o in run(frames, lead=lead))


def test_one_frame_flicker_does_not_trigger():
  frames = [{1: track(1, 20.0, 0.0, 5.0, 13.6)}, {}, {1: track(1, 19.5, 0.0, 5.0, 13.6)}, {}, {1: track(1, 19.0, 0.0, 5.0, 13.6)}]
  assert all(o is FAR_LEAD for o in run(frames))
