# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
import numpy as np

from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.long_mpc_ext import LongMpcObstacleExt, get_far_gap_extra

LP = log.LongitudinalPersonality


def test_far_gap_tables():
  v = np.array([50.0, 60.0, 80.0, 95.0, 105.0, 130.0, 150.0]) / 3.6
  assert np.allclose(get_far_gap_extra(v, LP.relaxed), [0.0, 0.0, 20.0 / 7 * 2, 10.0, 20.0 / 7 * 4.5, 20.0, 20.0])
  assert np.allclose(get_far_gap_extra(v, LP.standard), [0.0, 0.0, 0.0, 3.0, 5.0, 10.0, 10.0])
  assert np.allclose(get_far_gap_extra(v, LP.aggressive), [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # +1 m 30-60 km/h (10-27)
  assert np.allclose(get_far_gap_extra(np.array([10.0, 25.0, 30.0, 45.0, 65.0]) / 3.6, LP.aggressive), [0.0, 0.5, 1.0, 1.0, 0.5])
  assert np.allclose(get_far_gap_extra(v, None), 0.0)


def test_adjust_moves_the_obstacle_closer_by_the_extra():
  n = 13
  v_plan = np.full(n, 130.0 / 3.6)
  lead = np.column_stack([np.full(n, 70.0), np.full(n, 130.0 / 3.6)])
  x = np.full((n, 2), 120.0)
  a = LongMpcObstacleExt().adjust(x.copy(), lead, lead, v_plan, personality=LP.aggressive)
  b = LongMpcObstacleExt().adjust(x.copy(), lead, lead, v_plan, personality=LP.relaxed)
  c = LongMpcObstacleExt().adjust(x.copy(), lead, lead, v_plan, personality=LP.standard)
  assert np.allclose(a - b, 20.0) and np.allclose(a - c, 10.0)
