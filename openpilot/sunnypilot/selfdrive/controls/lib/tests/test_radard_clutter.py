from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.radard import match_vision_to_track, RADAR_TO_CAMERA


def vision(x, y, v, x_std=3.0, y_std=0.3, v_std=3.0):
  return NS(x=[x], y=[y], v=[v], xStd=[x_std], yStd=[y_std], vStd=[v_std])


def f7_vision(v_kph):
  # the model on the car reports leadsV3 xStd ~6e4 m, yStd ~3 m, vStd ~3e2..6e4 m/s, so the match is decided by y alone
  return vision(54.3 + RADAR_TO_CAMERA, 1.14, v_kph / 3.6, x_std=59874.0, y_std=3.1, v_std=305.0)


def track(d, y, v_lead, v_ego):
  return NS(dRel=d, yRel=y, vRel=v_lead - v_ego)


# C3X route 000000f7 2026-10-04 14:22:12.7: ego 53.5 km/h in a left curve, camera lead at x 54.3 (+1.52) m, y -1.1 (radar
# frame y -(-1.1)), 21 km/h; radar: the lead (two duplicate tracks) at 56.9 m / y -0.6 / 20 km/h and a roadside point
# at 60.6 m / y -1.2 / -2 km/h. The max-prob match is the roadside point (closest in y).
V_EGO = 53.5 / 3.6


def f7_tracks():
  return {
    1852: track(56.9, -0.6, 20.0 / 3.6, V_EGO),
    1859: track(56.9, -0.8, 19.0 / 3.6, V_EGO),
    1856: track(60.6, -1.2, -2.0 / 3.6, V_EGO),
  }


def test_f7_roadside_point_rejected():
  lead = f7_vision(21.0)
  tracks = f7_tracks()
  # the stock match picks the stationary point (nearest in y) and the 10 m/s gate lets it through
  assert match_vision_to_track(V_EGO, lead, {1856: tracks[1856]}) is tracks[1856]
  assert max(tracks.values(), key=lambda c: abs(c.yRel + lead.y[0])) is not tracks[1856]
  # with the moving radar lead at the same place corroborating the camera it is rejected
  assert match_vision_to_track(V_EGO, lead, tracks) is None


def test_f7_rejected_across_the_camera_speed_jitter():
  # the camera speed jittered 21-36 km/h in the phantom frames; every one is rejected now
  for v_kph in (21.0, 25.0, 28.6, 31.2, 33.4, 35.1):
    tracks = f7_tracks()
    assert match_vision_to_track(V_EGO, f7_vision(v_kph), tracks) is None


def test_stopped_lead_without_moving_track_kept():
  # a stopped car at 47 m with the camera speed noisy up to ~30 km/h: no moving track at that place, radar lead kept
  v_ego = 10.0
  t = track(47.0, 0.0, 0.2, v_ego)
  assert match_vision_to_track(v_ego, vision(48.5 + RADAR_TO_CAMERA, 0.1, 8.4), {1: t}) is t


def test_stopped_lead_kept_next_to_adjacent_lane_traffic():
  # a stopped car in my lane and a moving car in the next lane (3.5 m over) at the same range do not reject it
  v_ego = 10.0
  stopped = track(47.0, 0.0, 0.2, v_ego)
  passing = track(46.0, 3.5, 9.0, v_ego)
  assert match_vision_to_track(v_ego, vision(48.5 + RADAR_TO_CAMERA, 0.1, 8.4), {1: stopped, 2: passing}) is stopped


def test_stopped_lead_kept_when_moving_track_disagrees_with_camera():
  # a moving track at the lead's place that does not agree with the camera speed is not corroboration
  v_ego = 10.0
  stopped = track(47.0, 0.0, 0.2, v_ego)
  fast = track(46.0, 0.3, 25.0, v_ego)
  assert match_vision_to_track(v_ego, vision(48.5 + RADAR_TO_CAMERA, 0.1, 8.4), {1: stopped, 2: fast}) is stopped


def test_stationary_lead_with_stationary_camera_kept():
  # camera also sees the lead as stopped: the rule never applies, whatever else the radar sees
  v_ego = 5.0
  stopped = track(20.0, 0.0, 0.0, v_ego)
  moving = track(21.0, 0.2, 6.0, v_ego)
  assert match_vision_to_track(v_ego, vision(20.0 + RADAR_TO_CAMERA, 0.0, 0.5), {1: stopped, 2: moving}) is stopped


def test_moving_match_unaffected():
  # the rule only concerns a stationary best match; a moving best match is returned as before
  v_ego = 15.0
  moving = track(40.0, 0.0, 12.0, v_ego)
  stopped = track(43.0, 0.1, 0.0, v_ego)
  assert match_vision_to_track(v_ego, vision(40.0 + RADAR_TO_CAMERA, 0.0, 12.0), {1: moving, 2: stopped}) is moving
