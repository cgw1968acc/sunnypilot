import numpy as np
import pytest
from cereal import car, custom
from unittest.mock import MagicMock

from openpilot.sunnypilot.selfdrive.controls.lib.longcontrol_sp import (
  LongControlSP,
  _enter_stop_target,
  _hold_stop_target,
  _approach_active,
  _carry_active,
  _approach_accel_cap,
  _carry_accel_floor,
  _dropout_guard,
)
from openpilot.selfdrive.controls.lib.stop_and_go_helpers import should_release_stop_hold_for_departing_lead
from openpilot.selfdrive.controls.lib.stopping_guard import apply_low_speed_output_slew
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


class TestApproachActive:
  def test_triggers_approaching_at_speed(self):
    assert _approach_active(2.0, -0.08, 2.0)

  def test_no_trigger_below_1ms(self):
    assert not _approach_active(0.8, -0.08, 2.0)

  def test_no_trigger_when_enter_stop_active(self):
    assert not _approach_active(0.3, -0.08, 0.5)

  def test_no_trigger_none_dist(self):
    assert not _approach_active(2.0, -0.08, None)

  def test_no_trigger_weak_decel(self):
    assert not _approach_active(2.0, -0.02, 2.0)


class TestCarryActive:
  def test_triggers_mid_speed_range(self):
    assert _carry_active(0.8, -0.15, 3.5)

  def test_no_trigger_below_range(self):
    assert not _carry_active(0.4, -0.15, 2.5)

  def test_no_trigger_above_range(self):
    assert not _carry_active(1.5, -0.15, 2.5)

  def test_no_trigger_weak_decel(self):
    assert not _carry_active(0.8, -0.05, 2.5)

  def test_no_trigger_when_approach_active(self):
    assert not _carry_active(2.0, -0.08, 2.0)


class TestApproachAccelCap:
  def test_caps_harder_when_close(self):
    assert _approach_accel_cap(2.0, 0.7) < _approach_accel_cap(2.0, 3.0)

  def test_caps_harder_at_speed(self):
    assert _approach_accel_cap(6.0, 2.0) < _approach_accel_cap(1.5, 2.0)

  def test_none_dist_treated_as_zero(self):
    assert _approach_accel_cap(2.0, None) == _approach_accel_cap(2.0, 0.0)


class TestCarryAccelFloor:
  def test_floor_more_negative_when_close(self):
    assert _carry_accel_floor(0.8, 1.5) < _carry_accel_floor(0.8, 5.0)

  def test_none_dist_treated_as_zero(self):
    assert _carry_accel_floor(0.8, None) == _carry_accel_floor(0.8, 0.0)


class TestDropoutGuard:
  def test_holds_when_criteria_met(self):
    assert _dropout_guard(0.10, 0.2, 0.8, 0.85, -0.20, 0.20)

  def test_no_hold_speed_too_high(self):
    assert not _dropout_guard(0.30, 0.2, 0.8, 0.85, -0.20, 0.20)

  def test_no_hold_accel_too_light(self):
    assert not _dropout_guard(0.10, 0.2, 0.8, 0.85, -0.05, 0.20)

  def test_no_hold_stop_intent_expired(self):
    assert not _dropout_guard(0.10, 0.2, 0.8, 0.85, -0.20, 0.50)

  def test_no_hold_dist_grew_too_much(self):
    assert not _dropout_guard(0.10, 0.2, 2.0, 0.85, -0.20, 0.20)

  def test_no_hold_none_dist(self):
    assert not _dropout_guard(0.10, 0.2, None, 0.85, -0.20, 0.20)

  def test_no_hold_none_last_dist(self):
    assert not _dropout_guard(0.10, 0.2, 0.8, None, -0.20, 0.20)


class TestDepartingLeadRelease:
  def test_releases_when_lead_departing_at_standstill(self):
    assert should_release_stop_hold_for_departing_lead(
      output_should_stop=True, standstill=True,
      v_ego=0.0, v_ego_starting=0.3,
      lead_status=True, lead_v=0.5, lead_d_rel=3.0
    )

  def test_no_release_when_not_stopping(self):
    assert not should_release_stop_hold_for_departing_lead(
      output_should_stop=False, standstill=True,
      v_ego=0.0, v_ego_starting=0.3,
      lead_status=True, lead_v=0.5, lead_d_rel=3.0
    )

  def test_no_release_lead_too_slow(self):
    assert not should_release_stop_hold_for_departing_lead(
      output_should_stop=True, standstill=True,
      v_ego=0.0, v_ego_starting=0.3,
      lead_status=True, lead_v=0.05, lead_d_rel=3.0
    )

  def test_no_release_lead_too_close(self):
    assert not should_release_stop_hold_for_departing_lead(
      output_should_stop=True, standstill=True,
      v_ego=0.0, v_ego_starting=0.3,
      lead_status=True, lead_v=0.5, lead_d_rel=1.0
    )

  def test_no_release_no_lead(self):
    assert not should_release_stop_hold_for_departing_lead(
      output_should_stop=True, standstill=True,
      v_ego=0.0, v_ego_starting=0.3,
      lead_status=False, lead_v=0.5, lead_d_rel=3.0
    )

  def test_no_release_speed_too_high(self):
    assert not should_release_stop_hold_for_departing_lead(
      output_should_stop=True, standstill=False,
      v_ego=2.0, v_ego_starting=0.3,
      lead_status=True, lead_v=2.5, lead_d_rel=3.0
    )


class TestLowSpeedOutputSlew:
  def test_passthrough_above_12ms(self):
    result = apply_low_speed_output_slew(-0.5, -0.3, True, 1.5, -0.2, -0.2, False, False)
    assert result == -0.5

  def test_limits_brake_step(self):
    result = apply_low_speed_output_slew(-0.5, -0.3, True, 0.1, -0.2, -0.2, False, False)
    assert result > -0.5

  def test_limits_release_step(self):
    result = apply_low_speed_output_slew(0.1, -0.3, False, 0.1, -0.2, -0.2, False, False)
    assert result < 0.1

  def test_fast_release_allows_bigger_step(self):
    normal = apply_low_speed_output_slew(0.3, -0.3, False, 0.1, -0.2, -0.2, False, False)
    fast   = apply_low_speed_output_slew(0.3, -0.3, False, 0.1, -0.2, -0.2, True,  False)
    assert fast > normal


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

  def test_picks_minimum_candidate(self):
    val, _ = update_distance_to_stop_target_with_latch(-1.0, 0.0, 0.01, (3.0, 1.5))
    assert val == pytest.approx(1.5)


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

  def test_departing_lead_clears_stop_hold(self):
    from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
    CS = make_cs(v_ego=0.0, standstill=True)
    CS.cruiseState.standstill = True
    self.loc.update(True, CS, -0.5, True, [-3.0, 2.0])
    assert self.loc.long_control_state == LongCtrlState.stopping
    out = self.loc.update(
      True, CS, 0.5, True, [-3.0, 2.0],
      lead_status=True, lead_v=0.6, lead_d_rel=3.0
    )
    assert self.loc.long_control_state != LongCtrlState.stopping or out > -0.5

  def test_approach_mode_caps_accel(self):
    CS = make_cs(v_ego=2.0, a_ego=-0.05)
    self.loc.long_control_state = __import__(
      'openpilot.selfdrive.controls.lib.longcontrol', fromlist=['LongCtrlState']
    ).LongCtrlState.pid
    cap = _approach_accel_cap(2.0, 2.0)
    out = self.loc.update(True, CS, -0.08, False, [-3.0, 2.0], distance_to_stop_target_m=2.0)
    assert out <= cap + 1e-4

  def test_output_clipped_to_accel_limits(self):
    CS = make_cs(v_ego=10.0)
    out = self.loc.update(True, CS, 5.0, False, [-3.0, 2.0])
    assert out <= 2.0

  def test_reset_clears_timers(self):
    CS = make_cs(v_ego=0.1)
    self.loc.update(True, CS, -0.5, True, [-3.0, 2.0])
    self.loc.reset()
    assert self.loc.time_since_stop_intent_s == pytest.approx(10.0)
    assert self.loc.time_since_standstill_s  == pytest.approx(10.0)
    assert self.loc.last_dist is None
    assert self.loc._release_lock_counter == 0
    assert self.loc._was_off is True

  def test_engagement_seeded_to_a_ego_not_zero(self):
    # At highway speed, first pid frame after off should seed last_output_accel
    # to aEgo so the PID doesn't start from 0 (engagement jerk fix).
    # The seeded value is used as the slew baseline; the PID output itself
    # will still follow a_target via feedforward, but the key is _was_off
    # is consumed and last_output_accel was initialized to aEgo.
    CS = make_cs(v_ego=20.0, a_ego=-0.3)
    # First call is off — _was_off must stay True
    self.loc.update(False, CS, -0.1, False, [-3.0, 2.0])
    assert self.loc._was_off is True
    # Second call engages — _was_off consumed; output is not zero (would be 0
    # in old code since last_output_accel started at 0 and slew limited from there)
    out = self.loc.update(True, CS, -0.1, False, [-3.0, 2.0])
    assert self.loc._was_off is False
    # Output should not be stuck near 0 — it should be near a_target or aEgo
    assert out < -0.05

  def test_hold_stop_veto_bypassed_when_planner_cleared(self):
    # When effective_should_stop=False (planner said go, e.g. green light with
    # no lead), _hold_stop_target must NOT veto the stopping→pid transition.
    from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
    CS = make_cs(v_ego=0.1, a_ego=-0.1)
    # Put controller into stopping state
    self.loc.update(True, CS, -0.5, True, [-3.0, 2.0], distance_to_stop_target_m=0.8)
    assert self.loc.long_control_state == LongCtrlState.stopping
    # Planner clears should_stop (green light); no lead; dist still in hold range
    # Previously _hold_stop_target would re-lock; now it should not.
    self.loc.update(True, CS, -0.05, False, [-3.0, 2.0], distance_to_stop_target_m=0.8)
    assert self.loc.long_control_state != LongCtrlState.stopping

  def test_stopping_ramp_capped_at_floor(self):
    # After many frames in stopping, accel must not go below STOPPING_ACCEL_FLOOR
    # regardless of how long the stop lasts (prevents huge jerk on release).
    from openpilot.sunnypilot.selfdrive.controls.lib.longcontrol_sp import STOPPING_ACCEL_FLOOR
    CS = make_cs(v_ego=0.0, standstill=True)
    CS.cruiseState.standstill = True
    for _ in range(500):
      self.loc.update(True, CS, -0.5, True, [-3.0, 2.0])
    assert self.loc.last_output_accel >= STOPPING_ACCEL_FLOOR - 1e-6

  def test_stopping_exit_slew_limits_release_rate(self):
    # When leaving stopping with a wound-down accel, output must not jump to
    # PID output instantly — the exit slew should bound the per-frame rise.
    from openpilot.sunnypilot.selfdrive.controls.lib.longcontrol_sp import STOPPING_EXIT_RELEASE_RATE
    from openpilot.common.realtime import DT_CTRL
    CS = make_cs(v_ego=0.0, standstill=True)
    CS.cruiseState.standstill = True
    # Wind down the stopping ramp
    for _ in range(200):
      self.loc.update(True, CS, -0.5, True, [-3.0, 2.0])
    wound_down_accel = self.loc.last_output_accel
    assert wound_down_accel < -0.05
    # Now planner says go — first pid frame should be rate-limited
    CS2 = make_cs(v_ego=0.1, standstill=False, a_ego=-0.05)
    CS2.cruiseState.standstill = False
    out = self.loc.update(True, CS2, 0.3, False, [-3.0, 2.0])
    max_allowed_rise = wound_down_accel + STOPPING_EXIT_RELEASE_RATE * DT_CTRL + 1e-4
    assert out <= max_allowed_rise

  def test_fast_release_not_suppressed_after_planner_clears_stop(self):
    # Once should_stop is False (planner cleared), allow_fast_release suppression
    # should not apply even if time_since_stop_intent_s < 1.0.
    CS = make_cs(v_ego=0.05, a_ego=-0.1, standstill=True)
    CS.cruiseState.standstill = True
    # Build up stop state
    self.loc.update(True, CS, -0.5, True, [-3.0, 2.0])
    self.loc.time_since_stop_intent_s = 0.2
    self.loc.time_since_standstill_s = 0.6
    # Now planner says go (stop_active=False)
    CS2 = make_cs(v_ego=0.05, a_ego=-0.05, standstill=False)
    CS2.cruiseState.standstill = False
    out = self.loc.update(True, CS2, 0.3, False, [-3.0, 2.0])
    # Should be able to release — output should be > last stopping output
    assert out >= self.loc.last_output_accel - 0.005
