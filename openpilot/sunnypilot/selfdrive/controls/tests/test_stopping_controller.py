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
END = StoppingController.END_REQUEST


def cs(v_kph, standstill=False):
  return SimpleNamespace(vEgo=v_kph / 3.6, standstill=standstill)


def run(sc, state, car, a_target, prev, secs, stock=None):
  """step the controller for secs with a fixed plan; returns the last output"""
  out = prev
  for _ in range(int(secs / DT_CTRL)):
    _, out = sc.update(state, state, car, a_target, out, a_target if stock is None else stock, LIMITS, has_lead=True)
  return out


class TestEndOfStop:
  def test_below_3kph_a_braking_plan_is_held_at_the_template_force(self):
    sc = StoppingController(-2.0)
    out = run(sc, PID, cs(2.8), -0.40, -0.40, 0.5)
    assert abs(out - END) < 1e-6
    # the planner's own curve eases off: the request does not lighten (creep) ...
    out = run(sc, PID, cs(2.0), -0.20, out, 0.5)
    assert abs(out - END) < 1e-6
    # ... and firms up: the request does not firm (nod)
    out = run(sc, PID, cs(1.5), -0.90, out, 0.5)
    assert abs(out - END) < 1e-6

  def test_entry_is_rate_limited(self):
    sc = StoppingController(-2.0)
    _, out = sc.update(PID, PID, cs(2.8), -0.40, -0.40, -0.40, LIMITS, has_lead=True)
    assert out < END
    assert abs(out - (-0.40 + StoppingController.END_RATE * DT_CTRL)) < 1e-6

  def test_an_emergency_plan_still_passes(self):
    sc = StoppingController(-2.0)
    out = run(sc, PID, cs(2.5), -1.8, -0.40, 0.8)
    assert abs(out - (-1.8)) < 1e-6

  def test_crawl_follow_is_untouched(self):
    sc = StoppingController(-2.0)
    out = run(sc, PID, cs(2.0), -0.10, -0.10, 0.5)
    assert abs(out - (-0.10)) < 1e-6
    assert not sc.end_active

  def test_above_3kph_nothing_changes(self):
    sc = StoppingController(-2.0)
    out = run(sc, PID, cs(5.0), -0.40, -0.40, 0.5)
    assert abs(out - (-0.40)) < 1e-6

  def test_lead_moving_off_releases_the_hold(self):
    sc = StoppingController(-2.0)
    run(sc, PID, cs(2.0), -0.40, -0.40, 0.5)
    assert sc.end_active
    _, out = sc.update(PID, PID, cs(1.5), 0.50, END, 0.50, LIMITS, has_lead=True)
    assert not sc.end_active and abs(out - 0.50) < 1e-6

  def test_stopping_state_keeps_the_force_until_the_wheels_stop_then_holds(self):
    sc = StoppingController(-1.0)
    run(sc, PID, cs(2.0), -0.60, -0.60, 0.5)
    # 1 km/h, stopping state, the planner's curve has eased to -0.30: Kumar's lighter-follow would ease the request
    out = run(sc, STOPPING, cs(0.8), -0.30, END, 0.4)
    assert abs(out - END) < 1e-6
    # wheels stop: unchanged until the hold delay, then ramps to stopAccel at the hold rate
    out_at_stop = run(sc, STOPPING, cs(0.0, standstill=True), -0.30, out, 0.5)
    assert abs(out_at_stop - END) < 1e-6
    out_held = run(sc, STOPPING, cs(0.0, standstill=True), -0.30, out_at_stop, 1.0)
    assert out_held < out_at_stop - 0.3
    assert out_held >= -1.0 - 1e-6
