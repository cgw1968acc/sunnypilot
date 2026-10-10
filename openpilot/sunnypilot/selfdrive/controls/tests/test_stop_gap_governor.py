"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap_governor import StopGapGovernor, CREEP_ACCEL, GAP_MAX_DECEL, GAP_TARGET


def kph(v):
  return v / 3.6


def run(g, secs, out, v, d=None, vl=0.0, a_target=-0.5, braking=True, standstill=False):
  res = out
  for _ in range(int(round(secs / DT_CTRL))):
    res = g.update(out, kph(v), standstill, a_target, braking, d, vl if d is not None else None)
  return res


class TestStopGapGovernor:
  def test_on_track_nothing_changes(self):
    g = StopGapGovernor()
    # 10 km/h, 12 m behind a stopped lead: -0.43 needed, the plan asks -0.9
    assert run(g, 1.0, -0.9, 10, d=12.0) == -0.9

  def test_short_gap_brakes_firmer_smoothly(self):
    g = StopGapGovernor()
    # 7 km/h, 5.0 m behind a stopped lead: -1.35 needed to stop at GAP_TARGET, the plan asks -0.9
    need = -kph(7) ** 2 / (2 * (5.0 - GAP_TARGET))
    out = run(g, 0.1, -0.9, 7, d=5.0)
    assert -0.9 - 0.15 - 1e-6 <= out < -0.9  # entered from the plan at 1.5 m/s^3
    out = run(g, 1.0, -0.9, 7, d=5.0)
    assert abs(out - need) < 1e-6

  def test_capped(self):
    g = StopGapGovernor()
    assert abs(run(g, 3.0, -0.9, 10, d=3.0) - GAP_MAX_DECEL) < 1e-6

  def test_a_moving_lead_is_the_planners(self):
    g = StopGapGovernor()
    assert run(g, 1.0, -0.9, 10, d=5.0, vl=5.0) == -0.9

  def test_no_lead_no_gap_floor(self):
    g = StopGapGovernor()
    assert run(g, 1.0, -0.9, 7) == -0.9

  def test_releases_at_the_same_rate(self):
    g = StopGapGovernor()
    run(g, 1.0, -0.9, 7, d=5.0)
    out = run(g, 0.1, -0.9, 7, d=None)
    assert out < -0.9  # still easing back, no step
    assert run(g, 1.0, -0.9, 7, d=None) == -0.9

  def test_creep_at_the_end_is_stopped(self):
    g = StopGapGovernor()
    run(g, 0.5, -0.30, 0.8, d=3.7)
    assert run(g, 0.05, -0.30, 0.9, d=3.7) == -0.30   # +0.1 km/h: noise
    out = run(g, 0.2, -0.30, 1.05, d=3.7)              # +0.25 km/h: rolling in again
    assert abs(out - CREEP_ACCEL) < 1e-6
    out = run(g, 0.1, -0.30, 1.0, d=3.6)               # stays until the wheels stop
    assert abs(out - CREEP_ACCEL) < 1e-6
    assert run(g, 0.01, -0.30, 0.0, d=3.6, standstill=True) == -0.30
    assert not g.creep_stop

  def test_lead_moving_off_lets_go(self):
    g = StopGapGovernor()
    run(g, 1.0, -0.9, 7, d=5.0)
    assert run(g, 0.01, 0.3, 7, d=5.0, a_target=0.3) == 0.3
