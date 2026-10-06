import types
import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.curvature_shaping import (TurnExitSwingLimiter,
                                                                                        EXIT_COUNTER_CAP_V)

DT = 0.01
V = 30 / 3.6


def run(profile, v=V, plan=None, lane_changing=False):
  lim = TurnExitSwingLimiter(DT)
  out = []
  for lat in profile:
    k = lat / v ** 2
    mv = None if plan is None else types.SimpleNamespace(acceleration=types.SimpleNamespace(y=plan(len(out))))
    out.append(lim.update(k, k, v, True, lane_changing, mv) * v ** 2)
  return np.array(out)


def exit_profile():
  # 2 s left turn at -2.0, unwinding in 1 s, then the model swings to +0.33 and back (19:00:04)
  turn = [-2.0] * 200
  unwind = list(np.linspace(-2.0, 0.0, 100))
  swing = list(0.33 * np.sin(np.linspace(0, np.pi, 150))) + list(-0.16 * np.sin(np.linspace(0, np.pi, 150)))
  return np.array(turn + unwind + swing + [0.0] * 300)


def test_counter_swing_capped_after_a_turn():
  p = exit_profile()
  out = run(p)
  assert np.max(out[300:500]) <= EXIT_COUNTER_CAP_V[0] + 1e-9   # +0.33 -> 0.15
  assert np.max(p[300:500]) > 0.3
  np.testing.assert_allclose(out[:300], p[:300])                # the turn itself and its unwinding are untouched
  assert np.min(out[450:600]) < -0.15                           # the swing back toward the turn side is not capped


def test_released_after_the_window():
  p = exit_profile()
  p[700:] = 0.4
  out = run(p)
  assert abs(out[-1] - 0.4) < 1e-9


def test_real_s_bend_is_not_capped():
  p = exit_profile()
  def plan(i):   # the plan asks 1.0 m/s^2 the other way 1-3 s ahead
    return np.full(len(ModelConstants.T_IDXS), 1.0)
  out = run(p, plan=plan)
  np.testing.assert_allclose(out, p)


def test_off_at_speed_and_in_lane_change():
  p = exit_profile()
  np.testing.assert_allclose(run(p, v=60 / 3.6), p)
  np.testing.assert_allclose(run(p, lane_changing=True), p)


def test_no_turn_no_cap():
  p = 0.4 * np.sin(np.linspace(0, 6 * np.pi, 800))
  np.testing.assert_allclose(run(p), p)
