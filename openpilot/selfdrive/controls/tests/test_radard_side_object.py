from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.radard import match_vision_to_track, RADAR_TO_CAMERA


def vision(x, y, v):
  return NS(x=[x], y=[y], v=[v], xStd=[1.0], yStd=[1.0], vStd=[1.0])


def track(d, y, v_lead, v_ego):
  return NS(dRel=d, yRel=y, vRel=v_lead - v_ego)


def test_turn_misfire_rejected():
  # 2026-10-01 13:31:59: ego 20 km/h mid left turn, camera lead 11 m at 23 km/h (yRel +3.0), only radar point stationary 8.9 m left
  v_ego = 5.6
  assert match_vision_to_track(v_ego, vision(11.1, -3.0, 6.3), {1: track(9.4, 8.9, 0.1, v_ego)}) is None


def test_stopped_lead_kept_despite_noisy_camera_speed():
  # four-bookmark approach: camera speed noise up to ~8 m/s, but radar and camera agree laterally
  v_ego = 10.0
  t = track(47.0, 0.0, 0.2, v_ego)
  assert match_vision_to_track(v_ego, vision(48.5 + RADAR_TO_CAMERA, 0.1, 8.4), {1: t}) is t


def test_moving_lead_offset_kept():
  # a moving radar lead is never dropped by this rule, even with a lateral disagreement in a turn
  v_ego = 5.0
  t = track(10.0, 4.0, 6.0, v_ego)
  assert match_vision_to_track(v_ego, vision(10.0 + RADAR_TO_CAMERA, -1.0, 6.0), {1: t}) is t


def test_stationary_lead_aligned_kept():
  v_ego = 3.0
  t = track(8.0, 0.5, 0.0, v_ego)
  assert match_vision_to_track(v_ego, vision(8.0 + RADAR_TO_CAMERA, 0.0, 0.3), {1: t}) is t
