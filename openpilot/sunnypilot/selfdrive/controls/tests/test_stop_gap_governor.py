"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap_governor import (StopGapGovernor, CREEP_ACCEL, GAP_MAX_DECEL, GAP_TARGET,
                                                                           EASE_MAX, EASE_MIN_ACCEL, EASE_JERK)


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
    # 10 km/h, 7.9 m behind a stopped lead: -0.9 stops the car at GAP_TARGET, the plan asks -0.9
    assert run(g, 1.0, -0.9, 10, d=7.9) == -0.9
    # 0.8 m long: inside the ease margin, left alone
    assert run(g, 1.0, -0.9, 10, d=8.7) == -0.9

  def test_far_gap_eases_smoothly(self):
    g = StopGapGovernor()
    # 10 km/h, 12 m behind a stopped lead: -0.46 stops the car at GAP_TARGET, the plan asks -0.9 (stops 4 m short)
    out = run(g, 0.1, -0.9, 10, d=12.0)
    assert -0.9 < out <= -0.9 + EASE_JERK * 0.1 + 1e-6
    out = run(g, 2.0, -0.9, 10, d=12.0)
    assert abs(out - (-0.9 + EASE_MAX)) < 1e-6  # capped at EASE_MAX lighter

  def test_ease_aims_at_the_target_and_floor(self):
    g = StopGapGovernor()
    need = -kph(10) ** 2 / (2 * (9.5 - GAP_TARGET))  # -0.65
    assert abs(run(g, 3.0, -0.9, 10, d=9.5) - need) < 1e-6
    g = StopGapGovernor()
    assert abs(run(g, 3.0, -0.6, 10, d=30.0) - EASE_MIN_ACCEL) < 1e-6  # never lighter than EASE_MIN_ACCEL

  def test_ease_counts_a_rolling_lead_as_stopping_hard(self):
    g = StopGapGovernor()
    # route 00000134 15:32:13.1: 9.4 km/h, 9.9 m behind a lead at 3.5 km/h, the plan asked -0.93 (the car had
    # over-delivered -1.2..-1.4 for a second, so the request alone now stops it 2.8 m short)
    out = run(g, 3.0, -0.93, 9.4, d=9.9, vl=3.5 / 3.6)
    s = 9.9 - GAP_TARGET + (3.5 / 3.6) ** 2 / (2 * 4.0)
    assert abs(out - min(-kph(9.4) ** 2 / (2 * s), -0.93 + EASE_MAX)) < 1e-6

  def test_no_ease_for_a_moving_lead_or_no_lead(self):
    assert run(StopGapGovernor(), 2.0, -0.9, 10, d=20.0, vl=5.0) == -0.9
    assert run(StopGapGovernor(), 2.0, -0.9, 10) == -0.9
    assert run(StopGapGovernor(), 2.0, -0.9, 20, d=40.0) == -0.9  # above GAP_V_MAX

  def test_ease_lets_go_smoothly_when_the_gap_closes(self):
    g = StopGapGovernor()
    run(g, 2.0, -0.9, 10, d=12.0)
    out = run(g, 0.1, -0.9, 10, d=None)
    assert out > -0.9  # still easing back, no step
    assert run(g, 2.0, -0.9, 10, d=None) == -0.9

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


def test_stop_gap_lead_reads_a_real_radar_state():
  # 2026-10-11: stop_gap_lead read leadOne.status, which RadarState.LeadData does not have (it is `present`), so
  # controlsd crashed on the first engaged frame (LKAS fault). Use a real capnp message, not a stand-in.
  from openpilot.cereal import log
  from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

  class Ext:
    pass

  rs = log.RadarState.new_message()
  ext = Ext()
  ext.sm = type('SM', (), {'__getitem__': lambda self, k: rs.as_reader(), 'valid': {'radarState': True}})()
  assert ControlsExt.stop_gap_lead(ext) == {}
  rs.leadOne.present = True
  rs.leadOne.dRel = 8.0
  rs.leadOne.vLead = 1.0
  assert ControlsExt.stop_gap_lead(ext) == {'lead_d': 8.0, 'lead_v': 1.0}
  ext.sm.valid['radarState'] = False
  assert ControlsExt.stop_gap_lead(ext) == {}
