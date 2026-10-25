"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.lane_centering import LaneCentering, LINE_T_LOOK

DT = 0.01
X = [0., 2., 4., 6., 8., 10., 12., 15., 18., 21., 25., 30., 35., 40., 45., 50., 55., 60., 70., 80.]


def _model(offset=0.0, curv=0.0, prob=0.9):
  """lane centre at y = offset + curv / 2 * x^2 (model y: positive = right)"""
  centre = np.array([offset + 0.5 * curv * x ** 2 for x in X])
  left = SimpleNamespace(x=X, y=list(centre - 1.65))
  right = SimpleNamespace(x=X, y=list(centre + 1.65))
  return SimpleNamespace(laneLines=[left, left, right, right], laneLineProbs=[prob, prob, prob, prob])


def _run(lc, md, v, k_model, seconds, lane_changing=False):
  out = 0.0
  for _ in range(int(seconds / DT)):
    out = lc.update(md, v, True, False, 0.0, k_model, lane_changing)
  return out


def test_takes_over_with_confident_lines_and_pursues_the_centre():
  lc = LaneCentering(DT)
  v = 100 / 3.6
  out = _run(lc, _model(offset=0.2), v, 0.0, 3.0)  # car 0.2 m LEFT of centre, model goes straight
  assert lc.line_w == 1.0
  look = v * LINE_T_LOOK
  assert abs(out - 2 * 0.2 / (look ** 2 + 0.04)) < 1e-5  # steers right, pure pursuit


def test_follows_the_lane_curvature_not_the_model():
  lc = LaneCentering(DT)
  v = 100 / 3.6
  out = _run(lc, _model(curv=1 / 1000), v, 1 / 800, 3.0)  # model asks tighter than the lane
  assert abs((1 / 800 + out) - 1 / 1000) < 2e-5


def test_no_line_mode_below_30_kph_or_without_model_curvature():
  lc = LaneCentering(DT)
  _run(lc, _model(offset=0.2), 25 / 3.6, 0.0, 3.0)
  assert lc.line_w == 0.0
  lc2 = LaneCentering(DT)
  for _ in range(300):
    out = lc2.update(_model(offset=0.2), 100 / 3.6, True, False)
  assert lc2.line_w == 0.0 and out == lc2.correction


def test_hands_back_on_weak_lines_lane_change_or_far_target():
  v = 100 / 3.6
  for md, k_model, lane_changing in ((_model(prob=0.6), 0.0, False), (_model(), 0.0, True), (_model(curv=1 / 300), 0.0, False)):
    lc = LaneCentering(DT)
    _run(lc, _model(), v, 0.0, 3.0)
    assert lc.line_w == 1.0
    _run(lc, md, v, k_model, 0.6, lane_changing)
    assert lc.line_w == 0.0


def test_weight_rises_in_two_seconds():
  lc = LaneCentering(DT)
  _run(lc, _model(), 100 / 3.6, 0.0, 1.0)
  assert 0.45 < lc.line_w < 0.55
