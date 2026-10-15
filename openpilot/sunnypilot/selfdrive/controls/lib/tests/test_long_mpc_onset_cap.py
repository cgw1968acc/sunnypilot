import numpy as np

import openpilot.sunnypilot.selfdrive.controls.lib.long_mpc_ext as EXT

T = np.array([0.0, 0.0098, 0.039, 0.088, 0.156, 0.244, 0.351, 0.478, 0.625, 0.791, 0.977, 1.182, 1.406])


def plan(v0, a):
  return np.maximum(v0 + a * T, 0.0)


def steps(ext, n, x_obs, v_plan, closing=1.0, dyn=0.0):
  out = None
  for _ in range(n):
    out = ext.update_onset_cap(x_obs, v_plan, T[1], T, closing, dyn)
  return out


def test_no_cap_without_a_lead_or_outside_the_speed_band_or_when_not_braking():
  ext = EXT.LongMpcObstacleExt()
  assert steps(ext, 20, 50.0, plan(12.0, -1.0), closing=-10.0) is None           # fake far lead
  assert ext.min_accel(-3.5) == -3.5
  assert steps(EXT.LongMpcObstacleExt(), 20, 50.0, plan(22.0, -1.0)) is None     # 80 km/h: highway untouched
  assert steps(EXT.LongMpcObstacleExt(), 20, 50.0, plan(12.0, 0.0)) is None      # not braking


def test_cap_starts_light_and_grows_linearly_along_time_and_the_horizon():
  ext = EXT.LongMpcObstacleExt()
  a_min = steps(ext, 1, 300.0, plan(12.0, -1.0))                                 # first braking step, 43 km/h, lead far
  assert abs(a_min[0] - (-(EXT.CAP_A0 + EXT.CAP_RATE * EXT.CAP_DT))) < 1e-9
  assert a_min[-1] < a_min[0]                                                     # later nodes may brake more
  assert abs(a_min[-1] - (-(EXT.CAP_A0 + EXT.CAP_RATE * (EXT.CAP_DT + T[-1])))) < 1e-9
  a_min = steps(ext, 20, 300.0, plan(12.0, -1.0))                                # 1.05 s into the event
  assert abs(a_min[0] - (-(EXT.CAP_A0 + EXT.CAP_RATE * 21 * EXT.CAP_DT))) < 1e-9
  assert np.all(a_min >= -3.5)


def test_cap_never_sits_below_the_decel_needed_to_stop_short_of_the_obstacle():
  ext = EXT.LongMpcObstacleExt()
  a_min = steps(ext, 1, 80.0, plan(12.0, -1.0))                                  # 43 km/h, obstacle 80 m: need 0.94
  need = 12.0 ** 2 / (2.0 * (80.0 - EXT.CAP_STOP_D))
  assert abs(a_min[0] - (-EXT.CAP_NEED_MARGIN * need)) < 1e-9
  assert steps(EXT.LongMpcObstacleExt(), 1, 3.0, plan(12.0, -1.0))[0] == -3.5    # hopeless: stock limit


def test_lead_urgency_releases_the_cap_faster_and_the_event_ends_after_a_second():
  ext = EXT.LongMpcObstacleExt()
  calm = steps(ext, 10, 300.0, plan(12.0, -1.0), dyn=0.0)[0]
  urgent = steps(EXT.LongMpcObstacleExt(), 10, 300.0, plan(12.0, -1.0), dyn=1.0)[0]
  assert urgent < calm
  assert abs(urgent - (-(EXT.CAP_A0 + EXT.CAP_RATE_URGENT * 10 * EXT.CAP_DT))) < 1e-9
  steps(ext, 19, 300.0, plan(12.0, 0.0))                                         # 0.95 s not braking: event still open
  assert ext.cap_t_on > 0.0
  steps(ext, 1, 300.0, plan(12.0, 0.0))
  assert ext.cap_t_on == 0.0 and ext.min_accel(-3.5) == -3.5
  assert abs(steps(ext, 1, 300.0, plan(12.0, -1.0))[0] - (-(EXT.CAP_A0 + EXT.CAP_RATE * EXT.CAP_DT))) < 1e-9  # fresh event
