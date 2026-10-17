import math

from openpilot.cereal import log
import openpilot.sunnypilot.selfdrive.controls.lib.highway_plan_curvature as hpc
from openpilot.sunnypilot.selfdrive.controls.lib.highway_plan_curvature import HighwayPlanCurvature, plan_curvature

T = [0.0, 0.2, 0.5, 1.0, 1.5, 2.0]
V = 96.0 / 3.6


def model(k_plan, v=V):
  m = log.ModelDataV2.new_message()
  m.orientationRate.t = T
  m.orientationRate.z = [k_plan * v] * len(T)
  m.velocity.x = [v] * len(T)
  return m


def run(c, n, k_action, k_plan, v=V, active=True, lc_off=True):
  m = model(k_plan, v)
  out = k_action
  for _ in range(n):
    out = c.update(k_action, m, v, active, lc_off)
  return out


def test_plan_curvature_window():
  assert math.isclose(plan_curvature(model(0.0005)), 0.0005, rel_tol=1e-6)
  assert plan_curvature(log.ModelDataV2.new_message()) is None


def test_below_65_kph_passes_through():
  c = HighwayPlanCurvature()
  assert run(c, 300, 0.0008, 0.0002, v=60.0 / 3.6) == 0.0008


def test_steady_curve_is_kept_in_full():
  # the plan reads less than the action on a steady curve: the slow bias puts the full action back
  c = HighwayPlanCurvature()
  assert math.isclose(run(c, 3000, 0.0008, 0.0007), 0.0008, abs_tol=1e-6)


def test_fast_action_swing_is_reduced_with_no_lag():
  c = HighwayPlanCurvature()
  run(c, 3000, 0.0012, 0.0012)  # a curve (0.85 m/s^2 at 96 km/h; since 10-17 the straight edge is +1 deg of wheel)
  outs = []
  for i in range(700):  # action swings +-0.0003 at ~0.29 Hz, the plan stays steady
    k = 0.0012 + 0.0003 * math.sin(2 * math.pi * i * 0.01 / 3.5)
    outs.append(c.update(k, model(0.0012), V, True, True))
  swing = max(outs[-350:]) - min(outs[-350:])
  assert swing < 0.6 * 0.0006


def test_lane_change_fades_out_at_the_rate():
  c = HighwayPlanCurvature()
  run(c, 300, 0.0014, 0.0010)
  base = c.update(0.0016, model(0.0010), V, True, True)
  out = c.update(0.0016, model(0.0010), V, True, False)
  assert c.w < 1.0 and out > base  # one step toward the action, not a jump
  assert run(c, 120, 0.0016, 0.0010, lc_off=False) == 0.0016


def test_inactive_or_disabled_passes_through(monkeypatch):
  c = HighwayPlanCurvature()
  assert run(c, 300, 0.0008, 0.0002, active=False) == 0.0008
  monkeypatch.setattr(hpc, "ENABLED", False)
  assert run(HighwayPlanCurvature(), 300, 0.0008, 0.0002) == 0.0008


def test_straight_uses_the_original():
  # owner 10-17: the averaged curvature swayed slowly on straights -> on a straight the action is sent unchanged
  c = HighwayPlanCurvature()
  k_straight = 0.05 / V ** 2  # 0.05 m/s^2 of road curve: a straight
  assert run(c, 300, 0.0002, k_straight) == 0.0002
  assert c.w == 0.0


def test_entering_a_curve_blends_in_over_a_second():
  c = HighwayPlanCurvature()
  run(c, 300, 0.0, 0.0)
  k_curve = 0.85 / V ** 2
  run(c, 50, k_curve, k_curve)
  assert 0.4 < c.w < 0.6
  run(c, 60, k_curve, k_curve)
  assert c.w == 1.0


def test_straight_edge_is_half_a_degree_more_of_wheel():
  from openpilot.sunnypilot.selfdrive.controls.lib.highway_plan_curvature import curve_weight
  v = 100.0 / 3.6
  deg = math.radians(0.5) / (2.67 * 13.9)
  assert curve_weight(math.radians(0.75) / (2.67 * 13.9), v) == 0.0  # 0.75 deg wheel at 100 km/h: straight now
  assert curve_weight(math.radians(1.2) / (2.67 * 13.9), v) == 1.0   # 1.2 deg: curve
  assert curve_weight(0.10 / v ** 2 + deg, v) == 0.0 and curve_weight(0.20 / v ** 2 + deg, v) > 0.999
