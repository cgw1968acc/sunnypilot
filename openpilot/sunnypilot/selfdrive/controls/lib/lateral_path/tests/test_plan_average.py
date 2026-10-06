from types import SimpleNamespace

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.curvature_shaping import (PlanAverageSmoother, PLAN_AVG_MAX_LAT_ACCEL,
                                                                                       plan_average_curvature)

T = [0.0, 0.01, 0.04, 0.09, 0.16, 0.24, 0.35, 0.48, 0.62, 0.79, 0.98, 1.18, 1.41]


def md(plan_curv, action, v=28.0, lc=0):
  z = [k * v for k in plan_curv]
  return SimpleNamespace(orientationRate=SimpleNamespace(t=T, z=z), velocity=SimpleNamespace(x=[v] * len(T)),
                         action=SimpleNamespace(desiredCurvature=action), meta=SimpleNamespace(laneChangeState=lc))


def test_plan_average_is_forward_mean():
  plan = list(np.linspace(0.0, 1.2e-3, len(T)))
  avg = plan_average_curvature(md(plan, 0.0))
  assert 0.0 < avg < 0.7e-3


def test_off_below_70_kph_inactive_and_in_lane_change():
  sm = PlanAverageSmoother(0.05)
  m = md([1e-4] * len(T), 0.0)
  assert sm.update(0.0, m, 60 / 3.6, True, False) == 0.0
  assert sm.update(0.0, m, 100 / 3.6, False, False) == 0.0
  assert sm.update(0.0, md([1e-4] * len(T), 0.0, lc=1), 100 / 3.6, True, True) == 0.0


def test_steady_curve_keeps_the_action():
  sm = PlanAverageSmoother(0.05)
  out = None
  for _ in range(600):  # 30 s in a steady curve where the plan average sits 7% below the action
    out = sm.update(1.0e-3, md([0.93e-3] * len(T), 1.0e-3), 100 / 3.6, True, False)
  assert abs(out - 1.0e-3) < 0.02e-3


def test_change_is_capped():
  sm = PlanAverageSmoother(0.05)
  v = 100 / 3.6
  sm.update(0.0, md([0.0] * len(T), 0.0, v), v, True, False)
  out = sm.update(0.0, md([5e-3] * len(T), 0.0, v), v, True, False)   # a curve entry seen ahead
  assert abs(out) <= PLAN_AVG_MAX_LAT_ACCEL / v ** 2 + 1e-12
