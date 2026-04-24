import numpy as np
import pytest
from cereal import car, custom
from unittest.mock import MagicMock

from openpilot.sunnypilot.selfdrive.controls.lib.longcontrol_sp import (
  LongControlSP,
  _enter_stop_target,
  _hold_stop_target,
)
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import (
  get_distance_to_stopped_lead_target,
  update_distance_to_stop_target_with_latch,
  STOP_TARGET_LATCH_S,
)


def make_cp(stopping_decel_rate=0.3, stop_accel=-0.5, v_ego_starting=0.3, start_accel=0.5):
  CP = car.CarParams.new_message()
  CP.stoppingDecelRate = stopping_decel_rate
  CP.stopAccel = stop_accel
  CP.vEgoStarting = v_ego_starting
  CP.startAccel = start_accel
  CP.longitudinalTuning.kpBP = [0.0]
  CP.longitudinalTuning.kpV = [0.0]
  CP.longitudinalTuning.kiBP = [0.0]
  CP.longitudinalTuning.kiV = [0.0]
  return CP


def make_cs(v_ego=0.0, a_ego=0.0, brake_pressed=False, standstill=False, cruise_standstill=False):
  CS = MagicMock()
  CS.vEgo = v_ego
  CS.aEgo = a_ego
  CS.brakePressed = brake_pressed
  CS.standstill = standstill
  CS.cruiseState.standstill = cruise_standstill
  return CS


class TestEnterStopTarget:
  def test_none_dist(self):
    assert not _enter_stop_target(0.0, -0.5, None)

  def test_zero_dist(self):
    assert not _enter_stop_target(0.0, -0.5, 0.0)

  def test_very_small_dist(self):
    assert not _enter_stop_target(0.0, -0.5, 0.05)

  def test_triggers_at_low_speed_close_dist(self):
    assert _enter_stop_target(0.3, -0.08, 0.35)

  def test_no_trigger_weak_decel(self):
    assert not _enter_stop_target(3.0, -0.02, 1.5)

  def test_no_trigger_far_dist(self):
    assert not _enter_stop_target(0.5, -0.2, 5.0)


class TestHoldStopTarget:
  def test_delegates_to_enter(self):
    assert _hold_stop_target(0.3, -0.08, 0.5)

  def test_hold_hysteresis_low_speed(self):
    assert _hold_stop_target(0.5, -0.06, 1.0)

  def test_no_hold_high_speed(self):
    assert not _hold_stop_target(3.0, -0.08, 1.5)

  def test_no_hold_none_dist(self):
    assert not _hold_stop_target(0.5, -0.1, None)


class TestStopTargetHelpers:
  def test_zero_when_no_distance(self):
    assert get_distance_to_stopped_lead_target(np.zeros(5), np.zeros(5)) == 0.0

  def test_zero_when_beyond_max(self):
    assert get_distance_to_stopped_lead_target(np.zeros(5), np.full(5, 10.0)) == 0.0

  def test_returns_positive_for_valid_stopped_lead(self):
    assert get_distance_to_stopped_lead_target(np.zeros(5), np.full(5, 2.0)) > 0.0

  def test_fades_out_as_lead_speeds_up(self):
    d_slow = get_distance_to_stopped_lead_target(np.full(5, 0.5), np.full(5, 2.0))
    d_fast = get_distance_to_stopped_lead_target(np.full(5, 2.5), np.full(5, 2.0))
    assert d_slow > d_fast

  def test_latch_keeps_value_briefly(self):
    val, latch = update_distance_to_stop_target_with_latch(2.0, STOP_TARGET_LATCH_S, 0.01, (0.0,))
    assert val == pytest.approx(2.0)
    assert latch > 0.0

  def test_latch_expires(self):
    val, latch = update_distance_to_stop_target_with_latch(2.0, 0.0, 0.01, (0.0,))
    assert val == -1.0
    assert latch == 0.0

  def test_latch_resets_when_candidate_available(self):
    val, latch = update_distance_to_stop_target_with_latch(2.0, 0.2, 0.01, (3.0,))
    assert val == pytest.approx(3.0)
    assert latch == pytest.approx(STOP_TARGET_LATCH_S)

  def test_picks_maximum_candidate(self):
    val, _ = update_distance_to_stop_target_with_latch(-1.0, 0.0, 0.01, (3.0, 1.5))
    assert val == pytest.approx(3.0)


class TestLongControlSP:
  def setup_method(self):
    self.CP    = make_cp()
    self.CP_SP = custom.CarParamsSP.new_message()
    self.loc   = LongControlSP(self.CP, self.CP_SP)

  def test_off_when_inactive(self):
    from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
    CS = make_cs(v_ego=5.0)
    self.loc.update(False, CS, 0.0, False, [-3.0, 2.0])
    assert self.loc.long_control_state == LongCtrlState.off

  def test_output_zero_when_off(self):
    CS = make_cs(v_ego=5.0)
    out = self.loc.update(False, CS, 0.0, False, [-3.0, 2.0])
    assert out == 0.0

  def test_enters_stopping_on_should_stop(self):
    from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
    CS = make_cs(v_ego=0.1)
    self.loc.update(True, CS, -0.5, True, [-3.0, 2.0])
    assert self.loc.long_control_state == LongCtrlState.stopping

  def test_stopping_ramps_to_stop_accel(self):
    CS = make_cs(v_ego=0.1)
    self.loc.last_output_accel = 0.0
    out = self.loc.update(True, CS, -0.5, True, [-3.0, 2.0])
    assert out < 0.0

  def test_output_clipped_to_accel_limits(self):
    CS = make_cs(v_ego=10.0)
    out = self.loc.update(True, CS, 5.0, False, [-3.0, 2.0])
    assert out <= 2.0

  def test_reset_clears_state(self):
    CS = make_cs(v_ego=0.1)
    self.loc.update(True, CS, -0.5, True, [-3.0, 2.0])
    self.loc.reset()
    assert self.loc._was_off is True
    assert self.loc._stopping_exit_slew_active is False

  def test_engagement_seeded_to_a_ego_not_zero(self):
    # At highway speed, first pid frame after off should seed last_output_accel
    # to aEgo so the PID doesn't start from 0 (engagement jerk fix).
    CS = make_cs(v_ego=20.0, a_ego=-0.3)
    # First call is off — _was_off must stay True
    self.loc.update(False, CS, -0.1, False, [-3.0, 2.0])
    assert self.loc._was_off is True
    # Second call engages — _was_off consumed; output is not zero
    out = self.loc.update(True, CS, -0.1, False, [-3.0, 2.0])
    assert self.loc._was_off is False
    assert out < -0.05

  def test_hold_stop_veto_bypassed_when_planner_cleared(self):
    # When effective_should_stop=False (planner said go), _hold_stop_target
    # must NOT veto the stopping→pid transition.
    from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
    CS = make_cs(v_ego=0.1, a_ego=-0.1)
    self.loc.update(True, CS, -0.5, True, [-3.0, 2.0], distance_to_stop_target_m=0.8)
    assert self.loc.long_control_state == LongCtrlState.stopping
    # Planner clears should_stop; no lead; dist still in hold range
    self.loc.update(True, CS, -0.05, False, [-3.0, 2.0], distance_to_stop_target_m=0.8)
    assert self.loc.long_control_state != LongCtrlState.stopping

  def test_stopping_ramp_capped_at_floor(self):
    # After many frames in stopping at standstill, accel must not go below STOPPING_ACCEL_FLOOR.
    from openpilot.sunnypilot.selfdrive.controls.lib.longcontrol_sp import STOPPING_ACCEL_FLOOR
    CS = make_cs(v_ego=0.0, standstill=True)
    CS.cruiseState.standstill = True
    for _ in range(500):
      self.loc.update(True, CS, -0.5, True, [-3.0, 2.0])
    assert self.loc.last_output_accel >= STOPPING_ACCEL_FLOOR - 1e-6

  def test_stopping_preserves_entry_decel_at_speed(self):
    # When entering stopping state at higher speed, decel must NOT snap to the
    # floor — the car must keep braking at the PID entry level.
    from openpilot.sunnypilot.selfdrive.controls.lib.longcontrol_sp import STOPPING_ACCEL_FLOOR
    from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
    self.loc.long_control_state = LongCtrlState.pid
    self.loc.last_output_accel = -1.5
    CS = make_cs(v_ego=1.0)
    out = self.loc.update(True, CS, -1.5, True, [-3.0, 2.0])
    assert out < STOPPING_ACCEL_FLOOR - 0.5

  def test_stopping_exit_slew_limits_release_rate(self):
    # When leaving stopping with a wound-down accel, output must not jump to
    # PID output instantly — the exit slew should bound the per-frame rise.
    from openpilot.sunnypilot.selfdrive.controls.lib.longcontrol_sp import STOPPING_EXIT_RELEASE_RATE
    from openpilot.common.realtime import DT_CTRL
    CS = make_cs(v_ego=0.0, standstill=True)
    CS.cruiseState.standstill = True
    for _ in range(200):
      self.loc.update(True, CS, -0.5, True, [-3.0, 2.0])
    wound_down_accel = self.loc.last_output_accel
    assert wound_down_accel < -0.05
    CS2 = make_cs(v_ego=0.1, standstill=False, a_ego=-0.05)
    CS2.cruiseState.standstill = False
    out = self.loc.update(True, CS2, 0.3, False, [-3.0, 2.0])
    max_allowed_rise = wound_down_accel + STOPPING_EXIT_RELEASE_RATE * DT_CTRL + 1e-4
    assert out <= max_allowed_rise

  def test_planner_cleared_stop_transitions_to_pid(self):
    # Once should_stop is False (planner cleared), controller must leave stopping.
    from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
    CS = make_cs(v_ego=0.05, a_ego=-0.1, standstill=True)
    CS.cruiseState.standstill = True
    self.loc.update(True, CS, -0.5, True, [-3.0, 2.0])
    assert self.loc.long_control_state == LongCtrlState.stopping
    CS2 = make_cs(v_ego=0.4, a_ego=-0.05, standstill=False)
    CS2.cruiseState.standstill = False
    self.loc.update(True, CS2, 0.3, False, [-3.0, 2.0])
    assert self.loc.long_control_state != LongCtrlState.stopping
