import numpy as np

import openpilot.sunnypilot.selfdrive.controls.lib.long_mpc_ext as EXT

T = np.array([0.0, 0.0098, 0.039, 0.088, 0.156, 0.244, 0.351, 0.478, 0.625, 0.791, 0.977, 1.182, 1.406])


def lead_xv(x0, v0, a):
  v = np.maximum(v0 + a * T, 0.0)
  return np.column_stack((x0 + v0 * T + 0.5 * a * T ** 2, v))


def plan(v0, a):
  return np.maximum(v0 + a * T, 0.0)


def step(ext, n, lead, v_plan):
  out = 0.0
  for _ in range(n):
    out = ext.update_dynamic_gap(lead, v_plan, T[1])
  return out


def test_ordinary_following_and_gentle_lead_slow_downs_add_nothing():
  ext = EXT.LongMpcObstacleExt()
  assert step(ext, 100, lead_xv(25.0, 12.0, 0.0), plan(12.0, 0.0)) == 0.0      # steady following
  assert step(ext, 100, lead_xv(25.0, 12.0, -0.5), plan(12.0, -0.5)) == 0.0    # the lead coasts down
  assert step(ext, 100, lead_xv(30.0, 0.0, 0.0), plan(8.0, -1.0)) == 0.0       # a stopped lead ahead (SET- case)


def test_hard_lead_brake_adds_fast_and_scales_with_its_decel():
  ext = EXT.LongMpcObstacleExt()
  assert step(ext, 6, lead_xv(22.0, 9.4, -3.0), plan(9.4, 0.0)) == 1.0          # full within 0.3 s
  ext = EXT.LongMpcObstacleExt()
  assert abs(step(ext, 100, lead_xv(22.0, 9.4, -1.4), plan(9.4, -0.5)) - 0.5) < 1e-6   # half way -0.8..-2.0


def test_add_goes_away_slowly_once_the_lead_is_nearly_stopped_or_pulls_away():
  ext = EXT.LongMpcObstacleExt()
  step(ext, 10, lead_xv(22.0, 9.4, -3.0), plan(9.4, -1.0))
  f = step(ext, 1, lead_xv(8.0, 2.0, -3.0), plan(5.0, -1.5))                     # lead below 10 km/h
  assert abs(f - (1.0 - EXT.DYN_RATE_OFF * EXT.DYN_DT)) < 1e-9
  assert step(ext, 40, lead_xv(8.0, 2.0, -3.0), plan(5.0, -1.5)) == 0.0          # gone after 1.5 s
  ext = EXT.LongMpcObstacleExt()
  step(ext, 10, lead_xv(22.0, 9.4, -3.0), plan(9.4, -1.0))
  assert step(ext, 1, lead_xv(22.0, 11.0, -3.0), plan(9.4, -1.0)) < 1.0         # lead pulling away: fading


def test_adjust_adds_the_gap_by_the_planned_speed_only_while_the_lead_brakes():
  ext = EXT.LongMpcObstacleExt()
  ext.night_gap.update = lambda: None
  ext.night_gap.extra = lambda v: np.zeros_like(v)
  v_plan = plan(12.5, -1.0)                                                      # 45 km/h, planned down the horizon
  x = np.zeros((len(T), 2))
  calm = ext.adjust(x.copy(), lead_xv(25.0, 12.5, 0.0), lead_xv(60.0, 12.5, 0.0), v_plan, T[1], T)
  for _ in range(10):
    hard = ext.adjust(x.copy(), lead_xv(25.0, 12.5, -3.0), lead_xv(60.0, 12.5, -3.0), v_plan, T[1], T)
  assert abs(calm[0, 0] - hard[0, 0] - 5.0) < 1e-6                              # +5 m at 45 km/h (obstacle moved closer)
  # further down the horizon the lead's own light-onset margin differs too (it is slower), so only the lower bound holds
  assert np.all(calm[1:, 0] - hard[1:, 0] >= np.interp(v_plan[1:], EXT.DYN_GAP_BP, EXT.DYN_GAP_V) - 1e-6)
