"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.sunnypilot.selfdrive.controls.lib.stopping_controller import StoppingController

LIMITS = (-3.5, 2.0)
PID, STOPPING, OFF = LongCtrlState.pid, LongCtrlState.stopping, LongCtrlState.off
END_LO, END_HI = StoppingController.END_LO, StoppingController.END_HI


def cs(v_kph, standstill=False):
  return SimpleNamespace(vEgo=v_kph / 3.6, standstill=standstill)


def run(sc, state, car, a_target, prev, secs, stock=None):
  """step the controller for secs with a fixed plan; returns the last output"""
  out = prev
  for _ in range(int(secs / DT_CTRL)):
    _, out = sc.update(state, state, car, a_target, out, a_target if stock is None else stock, LIMITS, has_lead=True)
  return out


def blend(sc, v_kph):
  return sc._blend(v_kph / 3.6)


class TestEndOfStop:
  def test_the_line_runs_from_the_entry_request_through_5kph_to_the_rest_value(self):
    sc = StoppingController(-2.0)
    out = run(sc, PID, cs(9.9), -1.20, -1.20, 0.2)
    assert sc.end_active and abs(sc.a_entry - (-1.20)) < 1e-6
    # 7.5 km/h: halfway between the entry request and END_HI; the planner has eased to -0.30, the line is sent
    out = run(sc, PID, cs(7.5), -0.30, out, 0.6)
    assert abs(blend(sc, 7.5) - (-1.20 + (END_HI - -1.20) * 0.5)) < 1e-6
    assert abs(out - blend(sc, 7.5)) < 1e-6
    # 5 km/h: END_HI; below it a parabola: steep first, flat toward the rest value; the wheel stop: END_LO
    out = run(sc, PID, cs(5.0), -0.20, out, 0.4)
    assert abs(out - END_HI) < 1e-6
    out = run(sc, PID, cs(2.5), -0.20, out, 0.4)
    assert abs(out - (-0.43 + -0.53) / 2) < 1e-6
    assert abs(blend(sc, 4.0) - (-0.58)) < 1e-6 and abs(blend(sc, 1.0) - (-0.30)) < 1e-6
    assert abs(blend(sc, 0.5) - (-0.30)) < 1e-6   # 1-0 km/h unchanged
    out = run(sc, PID, cs(0.0), -0.20, out, 0.4)
    assert abs(out - END_LO) < 1e-6

  def test_the_line_is_a_floor_a_firmer_plan_still_passes(self):
    sc = StoppingController(-2.0)
    out = run(sc, PID, cs(9.9), -1.20, -1.20, 0.2)
    out = run(sc, PID, cs(5.0), -1.10, out, 0.5)
    assert abs(out - (-1.10)) < 1e-6
    out = run(sc, PID, cs(2.5), -1.8, out, 0.6)
    assert abs(out - (-1.8)) < 1e-6

  def test_a_light_plan_at_entry_starts_the_line_at_end_hi(self):
    sc = StoppingController(-2.0)
    light = END_HI + 0.1
    out = run(sc, PID, cs(7.0), light, light, 0.5)
    assert abs(sc.a_entry - END_HI) < 1e-6
    assert abs(out - END_HI) < 1e-6
    # the planner's own curve eases off below 5 km/h: the request follows the line, never the plan
    out = run(sc, PID, cs(2.5), -0.10, out, 0.5)
    assert abs(out - blend(sc, 2.5)) < 1e-6

  def test_entry_is_rate_limited(self):
    sc = StoppingController(-2.0)
    light = END_HI + 0.2
    _, out = sc.update(PID, PID, cs(7.0), light, light, light, LIMITS, has_lead=True)
    assert out > END_HI
    assert abs(out - (light - StoppingController.END_RATE * DT_CTRL)) < 1e-6

  def test_an_emergency_plan_still_passes(self):
    sc = StoppingController(-2.0)
    out = run(sc, PID, cs(2.5), -1.8, -0.40, 0.8)
    assert abs(out - (-1.8)) < 1e-6

  def test_crawl_follow_is_untouched(self):
    sc = StoppingController(-2.0)
    out = run(sc, PID, cs(2.0), -0.10, -0.10, 0.5)
    assert abs(out - (-0.10)) < 1e-6
    assert not sc.end_active

  def test_above_blend_v_nothing_changes(self):
    sc = StoppingController(-2.0)
    out = run(sc, PID, cs(12.0), -0.40, -0.40, 0.5)
    assert abs(out - (-0.40)) < 1e-6
    assert not sc.end_active

  def test_lead_moving_off_releases_the_hold(self):
    sc = StoppingController(-2.0)
    run(sc, PID, cs(2.0), -0.60, -0.60, 0.5)
    assert sc.end_active
    _, out = sc.update(PID, PID, cs(1.5), 0.50, END_LO, 0.50, LIMITS, has_lead=True)
    assert not sc.end_active and abs(out - 0.50) < 1e-6

  def test_stopping_state_keeps_the_line_until_the_wheels_stop_then_holds(self):
    sc = StoppingController(-1.0)
    run(sc, PID, cs(2.0), END_HI, END_HI, 0.5)
    # 1 km/h, stopping state, the planner's curve has eased to -0.10: Kumar's lighter-follow would ease the request
    out = run(sc, STOPPING, cs(1.0), -0.10, blend(sc, 2.0), 0.4)
    assert abs(out - blend(sc, 1.0)) < 1e-6
    # wheels stop: the last value of the line is held until the hold delay, then ramps to stopAccel at the hold rate
    out_at_stop = run(sc, STOPPING, cs(0.0, standstill=True), -0.10, out, 0.9)
    assert abs(out_at_stop - out) < 1e-6 and END_HI < out_at_stop <= END_LO + 1e-6
    out_held = run(sc, STOPPING, cs(0.0, standstill=True), -0.10, out_at_stop, 1.0)
    assert out_held < out_at_stop - 0.3
    assert out_held >= -1.0 - 1e-6


class TestHoldRelease:
  """2026-10-04 14:43: leaving the hold is fast while the request is far below zero, then linear and slow through the
  hand-over band, and the plan is never exceeded."""
  def test_release_is_slow_through_the_handover_band(self):
    sc = StoppingController(-2.0)
    out = -2.0
    t = 0.0
    t_lo = t_hi = None
    state = STOPPING  # controlsd feeds the returned state back as prev_state (the 0.2 s exit debounce lives there)
    for _ in range(300):
      state, out = sc.update(state, PID, cs(0.0, standstill=True), 0.8, out, 0.8, LIMITS, has_lead=True)
      t += DT_CTRL
      if t_lo is None and out >= StoppingController.RELEASE_BAND_LO:
        t_lo = t
      if t_hi is None and out >= StoppingController.RELEASE_BAND_HI:
        t_hi = t
      assert out <= 0.8 + 1e-9
    assert t_lo is not None and t_lo < 0.6           # 0.2 s debounce, then -2.0 -> -0.6 at the fast rate (0.35 s)
    assert t_hi is not None and 1.15 < t_hi < 1.3    # then -0.6 -> +0.4 at 1.5 m/s^3 (0.67 s)
    assert abs(out - 0.8) < 1e-9 and not sc.releasing

  def test_a_plan_that_drops_again_ends_the_release(self):
    sc = StoppingController(-2.0)
    state, out = STOPPING, -2.0
    for _ in range(30):  # through the exit debounce
      state, out = sc.update(state, PID, cs(0.0, standstill=True), 0.5, out, 0.5, LIMITS, has_lead=True)
    assert sc.releasing and -2.0 < out < 0.5
    _, out = sc.update(state, PID, cs(0.0, standstill=True), -2.0, out, -2.0, LIMITS, has_lead=True)
    assert abs(out - (-2.0)) < 1e-9 and not sc.releasing
