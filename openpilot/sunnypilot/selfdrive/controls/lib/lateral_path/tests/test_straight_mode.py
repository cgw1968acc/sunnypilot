from openpilot.cereal import log
import openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.straight_mode as sm
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.straight_mode import StraightMode
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.curvature_shaping import DesiredCurvatureJerkLimiter
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.lane_centering import LaneCentering

DT = 0.01
T = [0.0, 0.5, 1.0, 2.0, 3.0]
V = 80.0 / 3.6


def model(road_lat_accel, v=V):
  m = log.ModelDataV2.new_message()
  m.orientationRate.t = T
  m.orientationRate.z = [road_lat_accel / v] * len(T)  # yaw rate = a / v
  m.velocity.x = [v] * len(T)
  return m


def run(s, n, road, v=V, active=True, lc=False):
  w = 0.0
  for _ in range(n):
    w = s.update(model(road, v), v, active, lc)
  return w


def test_straight_and_curve():
  assert run(StraightMode(DT), 150, 0.05) == 1.0
  assert run(StraightMode(DT), 150, 0.18) == 1.0  # 0.18 m/s^2 at 80 km/h (~0.6 deg wheel): straight since the +0.5 deg
  assert run(StraightMode(DT), 150, 0.40) == 0.0
  assert run(StraightMode(DT), 150, 0.05, v=60.0 / 3.6) == 0.0  # below 65 km/h: normal


def test_blends_over_a_second_and_off_in_lane_change_or_inactive():
  s = StraightMode(DT)
  assert 0.45 < run(s, 50, 0.05) < 0.55
  assert run(s, 200, 0.05, lc=True) == 0.0
  assert run(StraightMode(DT), 150, 0.05, active=False) == 0.0


def test_disabled(monkeypatch):
  monkeypatch.setattr(sm, "ENABLED", False)
  assert run(StraightMode(DT), 150, 0.05) == 0.0


def test_jerk_cap_opens_on_a_straight():
  normal, straight = DesiredCurvatureJerkLimiter(DT), DesiredCurvatureJerkLimiter(DT)
  a = normal.update(1e-3, V, True, False)
  b = straight.update(1e-3, V, True, False, straight=1.0)
  assert b > 5 * a  # 8 m/s^3 vs 1 m/s^3 at 80 km/h


def test_lane_centering_faster_on_a_straight():
  normal, fast = LaneCentering(DT), LaneCentering(DT)
  normal.update(None, V, True, False)
  fast.update(None, V, True, False, straight=1.0)
  assert fast.offset_filter.alpha > normal.offset_filter.alpha
