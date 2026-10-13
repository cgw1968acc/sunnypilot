import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.long_mpc_ext import get_stop_match_trim, LongMpcObstacleExt


def test_low_speed_equals_aggressive_distance():
  v = np.array([0.0, 2.0, 5.0, 30 / 3.6])
  for t in (1.75, 1.45):
    # desired distance t*v - trim == 1.25*v below 30 km/h
    assert np.allclose(t * v - get_stop_match_trim(v, t), 1.25 * v)


def test_fades_out_and_aggressive_untouched():
  v = np.array([45 / 3.6, 60 / 3.6, 100 / 3.6])
  assert np.allclose(get_stop_match_trim(v, 1.75), 0.0)
  assert np.allclose(get_stop_match_trim(np.array([1.0, 5.0, 8.0]), 1.25), 0.0)
  mid = get_stop_match_trim(np.array([37.5 / 3.6]), 1.75)[0]
  assert abs(mid - 0.5 * 37.5 / 3.6 * 0.5) < 1e-9


def test_adjust_applies_it_only_with_t_follow():
  ext = LongMpcObstacleExt()
  n = 5
  xo = np.full((n, 2), 50.0)
  lead = np.zeros((n, 2))
  v = np.full(n, 4.0)
  base = ext.adjust(xo, lead, lead, v)
  rel = ext.adjust(xo, lead, lead, v, 1.75)
  agg = ext.adjust(xo, lead, lead, v, 1.25)
  assert np.allclose(rel - base, 0.5 * 4.0)
  assert np.allclose(agg, base)
