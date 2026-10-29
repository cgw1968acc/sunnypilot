# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
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
    out = ext.update_emergency(lead, v_plan, T[1], T)
  return out


def test_ordinary_stop_is_untouched():
  # the lead brakes, but our plan stays lighter than EMERG_PLAN_A0: ordinary stops still end 3-3.5 m
  ext = EXT.LongMpcObstacleExt()
  assert step(ext, 100, lead_xv(25.0, 10.0, -2.2), plan(12.0, -1.8)) == 0.0


def test_hard_lead_brake_ramps_the_trim_in_and_keeps_it():
  # route 00000119 09:23-like: lead -2.2, plan -2.6 -> full EMERG_TRIM, built at EMERG_RATE_ON (no step)
  ext = EXT.LongMpcObstacleExt()
  trims = [ext.update_emergency(lead_xv(23.0, 11.0, -2.2), plan(12.8, -2.6), T[1], T) for _ in range(40)]
  steps = np.diff([0.0] + trims)
  assert max(steps) <= EXT.EMERG_RATE_ON * EXT.EMERG_DT + 1e-9
  assert abs(trims[-1] - EXT.EMERG_TRIM) < 1e-9
  # the plan softens (the trim worked): it is latched, not taken back
  assert abs(step(ext, 20, lead_xv(15.0, 4.0, -2.0), plan(8.0, -1.6)) - EXT.EMERG_TRIM) < 1e-9


def test_partial_trim_between_plan_thresholds():
  ext = EXT.LongMpcObstacleExt()
  mid = (EXT.EMERG_PLAN_A0 + EXT.EMERG_PLAN_A1) / 2
  assert abs(step(ext, 100, lead_xv(25.0, 10.0, -2.0), plan(12.0, mid)) - EXT.EMERG_TRIM / 2) < 1e-6


def test_released_when_the_lead_calms_or_we_stop():
  ext = EXT.LongMpcObstacleExt()
  step(ext, 40, lead_xv(23.0, 11.0, -2.2), plan(12.8, -2.6))
  # the lead stops braking and we are not closing: after EMERG_CALM_T it fades at EMERG_RATE_OFF
  calm = [ext.update_emergency(lead_xv(20.0, 10.0, 0.0), plan(10.0, 0.0), T[1], T) for _ in range(200)]
  assert calm[int(EXT.EMERG_CALM_T / EXT.EMERG_DT) - 2] == EXT.EMERG_TRIM
  assert calm[-1] == 0.0 and max(-np.diff(calm)) <= EXT.EMERG_RATE_OFF * EXT.EMERG_DT + 1e-9
  ext2 = EXT.LongMpcObstacleExt()
  step(ext2, 40, lead_xv(23.0, 11.0, -2.2), plan(12.8, -2.6))
  step(ext2, 100, lead_xv(3.0, 0.0, 0.0), plan(0.0, 0.0))  # standing
  assert not ext2.emerg and ext2.emerg_trim == 0.0


def test_harder_lead_keeps_more_room():
  # 10-27: the trim shrinks as the lead itself brakes harder (route 73 17:16, lead -2.7: sim 1.65 -> 2.33 m)
  for a_lead, want in ((-2.2, EXT.EMERG_TRIM), (-2.6, (EXT.EMERG_TRIM + EXT.EMERG_TRIM_HARD) / 2), (-3.5, EXT.EMERG_TRIM_HARD)):
    ext = EXT.LongMpcObstacleExt()
    assert abs(step(ext, 60, lead_xv(23.0, 11.0, a_lead), plan(12.8, -2.6)) - want) < 1e-6, a_lead
