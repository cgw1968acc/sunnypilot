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


def test_cap_starts_light_and_grows_along_the_horizon():
  ext = EXT.LongMpcObstacleExt()
  a_min = steps(ext, 1, 300.0, plan(12.0, -1.0))                                 # first braking step, 43 km/h, lead far
  assert abs(a_min[0] - (-EXT.CAP_A0)) < 1e-9
  assert np.all(np.diff(a_min) <= 1e-12) and a_min[-1] < a_min[0]               # later nodes may brake more
  # the growth along the horizon starts at CAP_RATE_START (capped by the calm rate) and never steps
  assert abs((a_min[0] - a_min[1]) / T[1] - min(EXT.CAP_RATE_START, EXT.CAP_RATE)) < 0.01
  assert np.all(a_min >= -3.5)


def test_cap_follows_the_brake_in_use_after_a_long_light_brake():
  # route 000000ba 21:28:46: 11 s of light braking (-0.2..-0.4) before the lead braked hard; the old cap had grown
  # with the event time and was gone. Now it stays CAP_HEAD above what the plan uses.
  ext = EXT.LongMpcObstacleExt()
  a_min = steps(ext, 220, 300.0, plan(12.0, -0.4))
  assert ext.cap_level <= max(EXT.CAP_A0, 0.4 + EXT.CAP_HEAD) + 1e-9
  assert a_min[0] >= -(max(EXT.CAP_A0, 0.4 + EXT.CAP_HEAD) + 1e-9)


def test_pressing_against_the_cap_builds_the_rate_smoothly():
  for dyn, top in ((0.0, EXT.CAP_RATE), (1.0, EXT.CAP_RATE_URGENT)):
    ext = EXT.LongMpcObstacleExt()
    levels = []
    for _ in range(80):  # the plan always wants more than the cap: it sits on it
      steps(ext, 1, 300.0, plan(12.0, -(ext.cap_level if ext.cap_level is not None else EXT.CAP_A0)), dyn=dyn)
      levels.append(ext.cap_level)
    rates = np.diff(levels) / EXT.CAP_DT
    assert rates[0] <= max(EXT.CAP_RATE_START, top) + 1e-6
    assert np.all(np.diff(rates) >= -1e-9)                                       # the rate only builds up ...
    assert np.all(np.diff(rates) <= EXT.CAP_RATE_RAMP * EXT.CAP_DT + 1e-9)       # ... at CAP_RATE_RAMP (no jerk step)
    assert rates[-1] <= max(top, EXT.CAP_RATE_START) + 1e-9


def test_cap_never_sits_below_the_decel_needed_to_stop_short_of_the_obstacle():
  ext = EXT.LongMpcObstacleExt()
  a_min = steps(ext, 1, 80.0, plan(12.0, -1.0))                                  # 43 km/h, obstacle 80 m: need 0.94
  need = 12.0 ** 2 / (2.0 * (80.0 - EXT.CAP_STOP_D - 12.0 * EXT.CAP_REACT_T))    # less the distance the response takes
  assert abs(a_min[0] - (-EXT.CAP_NEED_MARGIN * need)) < 1e-9
  # a hard-braking lead (urgency 1): the stop may end CAP_STOP_D_URGENT short
  a_min = steps(EXT.LongMpcObstacleExt(), 1, 80.0, plan(12.0, -1.0), dyn=1.0)
  assert abs(a_min[0] - (-EXT.CAP_NEED_MARGIN * 12.0 ** 2 / (2.0 * (80.0 - EXT.CAP_STOP_D_URGENT - 12.0 * EXT.CAP_REACT_T)))) < 1e-9
  assert steps(EXT.LongMpcObstacleExt(), 1, 3.0, plan(12.0, -1.0))[0] == -3.5    # hopeless: stock limit


def test_the_event_ends_after_a_second_without_braking():
  ext = EXT.LongMpcObstacleExt()
  steps(ext, 10, 300.0, plan(12.0, -1.0))
  steps(ext, 19, 300.0, plan(12.0, 0.0))                                         # 0.95 s not braking: event still open
  assert ext.cap_t_on > 0.0
  steps(ext, 1, 300.0, plan(12.0, 0.0))
  assert ext.cap_t_on == 0.0 and ext.min_accel(-3.5) == -3.5 and ext.cap_level is None
  assert abs(steps(ext, 1, 300.0, plan(12.0, -1.0))[0] - (-EXT.CAP_A0)) < 1e-9  # fresh event


def test_floor_uses_where_a_moving_lead_itself_stops():
  # behind a steady lead (no decel) the floor uses CAP_LEAD_A_MIN, not the stock 2.5 m/s^2 stopped equivalence
  ext = EXT.LongMpcObstacleExt()
  n = len(T)
  lead = np.column_stack([16.0 + 9.4 * T, np.full(n, 9.4)])                       # 34 km/h lead 16 m ahead, steady
  x_obs = np.column_stack([lead[:, 0] + lead[:, 1] ** 2 / 5.0, np.full(n, 300.0)])
  ext.adjust(x_obs, lead, np.column_stack([np.full(n, 300.0), np.zeros(n)]), plan(9.6, -0.4), T[1], T,
             EXT.log.LongitudinalPersonality.aggressive)
  assert ext.a_min is not None and ext.a_min[0] > -1.0                           # soft: the stock equivalence asked ~-2.0
