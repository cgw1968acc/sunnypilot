from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import custom
from openpilot.selfdrive.controls.lib.drive_helpers import STOPPING_SPEED, should_stop
from openpilot.selfdrive.controls.lib.longcontrol import STOPPING_DECEL_RATE, STANDSTILL_HOLD_RATE, STANDSTILL_HOLD_DELAY_LEAD, STANDSTILL_HOLD_DELAY_NO_LEAD, STOPPING_FREEZE_MAX, STOPPING_EXIT_DEBOUNCE, STOPPING_FOLLOW_MIN, END_TAPER_V_START, end_taper_decel, CREEP_RATE_BASE, LongControl, \
  LongCtrlState, long_control_state_trans


class TestLongControlStateTransition(OpenpilotTestCase):

  def test_stay_stopped(self):
    CP_SP = custom.CarParamsSP.new_message()
    active = True
    current_state = LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid
    active = False
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.off

  def test_engage(self):
    CP_SP = custom.CarParamsSP.new_message()
    active = True
    current_state = LongCtrlState.off
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid

class TestTerminalStop(OpenpilotTestCase):
  def test_stopping_tune_is_gentler_than_upstream_default(self):
    # Upstream #38394 hardcoded a 1.0 m/s^2/s ramp and a 0.3 m/s latch. comma's own one-stopping-tune uses
    # 0.3 / 0.25, and every stop recorded on this car was driven with that pair. Both must stay on the less
    # braking side, or a future edit re-deepens the terminal brake unnoticed - which already happened once.
    assert 0.0 < STOPPING_DECEL_RATE <= 1.0
    assert 0.0 < STOPPING_SPEED <= 0.3
    assert should_stop(STOPPING_SPEED - 0.01, 0.0)
    assert not should_stop(0.29, 0.0)  # the band upstream would latch in and we do not

  def test_standstill_hold_firms_faster_than_the_stopping_ramp(self):
    # the hold only speeds up once the car is stopped, and reaches a firm -1.0 m/s^2 hold in about a second
    assert STANDSTILL_HOLD_RATE >= STOPPING_DECEL_RATE
    assert 0.5 <= STANDSTILL_HOLD_RATE <= 2.0

  def test_standstill_hold_waits_after_the_standstill_flag(self):
    """The request is frozen through the stop and for the hold delay after CS.standstill; only then it firms.
    0.5 s with a lead, 0.8 s without one."""
    from opendbc.car.structs import car
    from openpilot.common.realtime import DT_CTRL
    for has_lead, delay in ((True, STANDSTILL_HOLD_DELAY_LEAD), (False, STANDSTILL_HOLD_DELAY_NO_LEAD)):
      with self.subTest(has_lead=has_lead):
        CP = car.CarParams.new_message(stopAccel=-1.0)
        CP_SP = custom.CarParamsSP.new_message()
        LoC = LongControl(CP, CP_SP)
        moving = car.CarState.new_message(vEgo=0.2, standstill=False)
        still = car.CarState.new_message(vEgo=0.0, standstill=True)
        LoC.long_control_state = LongCtrlState.stopping
        LoC.last_output_accel = -0.3
        # 0.5 s of stopping state before the wheels read zero: nothing changes
        for _ in range(int(0.5 / DT_CTRL)):
          a = float(LoC.update(True, moving, 0.0, True, (-3.5, 1.5), has_lead=has_lead))
        assert a == -0.3
        a = [-0.3]
        for _ in range(int(2.5 / DT_CTRL)):
          a.append(float(LoC.update(True, still, 0.0, True, (-3.5, 1.5), has_lead=has_lead)))
        n_delay = int(delay / DT_CTRL)
        assert a[n_delay - 1] == -0.3                              # still frozen at the end of the wait
        assert a[n_delay + 5] < -0.3                               # and falling right after it
        late = (a[n_delay] - a[n_delay + 50]) / (50 * DT_CTRL)
        assert abs(late - STANDSTILL_HOLD_RATE) < 0.05, late      # fast once the delay has passed
    assert STANDSTILL_HOLD_DELAY_NO_LEAD > STANDSTILL_HOLD_DELAY_LEAD

  def test_stopping_ramp_resumes_if_the_car_never_stops(self):
    from opendbc.car.structs import car
    from openpilot.common.realtime import DT_CTRL
    CP = car.CarParams.new_message(stopAccel=-1.0)
    LoC = LongControl(CP, custom.CarParamsSP.new_message())
    LoC.long_control_state = LongCtrlState.stopping
    LoC.last_output_accel = -0.1
    creeping = car.CarState.new_message(vEgo=0.15, standstill=False)
    for _ in range(int(STOPPING_FREEZE_MAX / DT_CTRL) - 1):
      a = float(LoC.update(True, creeping, 0.0, True, (-3.5, 1.5)))
    assert a == -0.1                                           # frozen for the whole grace period
    for _ in range(int(1.0 / DT_CTRL)):
      a = float(LoC.update(True, creeping, 0.0, True, (-3.5, 1.5)))
    assert abs((-0.1 - a) - STOPPING_DECEL_RATE) < 0.02        # then the upstream ramp

  def test_standstill_hold_timer_resets_when_the_car_moves(self):
    from opendbc.car.structs import car
    CP = car.CarParams.new_message(stopAccel=-1.0)
    LoC = LongControl(CP, custom.CarParamsSP.new_message())
    LoC.long_control_state = LongCtrlState.stopping
    LoC.last_output_accel = -0.3
    still = car.CarState.new_message(standstill=True)
    moving = car.CarState.new_message(standstill=False, vEgo=0.1)
    for _ in range(40):
      LoC.update(True, still, 0.0, True, (-3.5, 1.5))
    LoC.update(True, moving, 0.0, True, (-3.5, 1.5))
    assert LoC.standstill_t == 0.0

  def test_one_frame_go_does_not_release_the_hold_at_standstill(self):
    from opendbc.car.structs import car
    from openpilot.common.realtime import DT_CTRL
    CP = car.CarParams.new_message(stopAccel=-1.0)
    CP.longitudinalTuning.kiBP = [0.0]
    CP.longitudinalTuning.kiV = [0.0]
    LoC = LongControl(CP, custom.CarParamsSP.new_message())
    still = car.CarState.new_message(vEgo=0.0, standstill=True)
    LoC.long_control_state = LongCtrlState.stopping
    LoC.last_output_accel = -1.0
    for _ in range(200):
      LoC.update(True, still, 0.0, True, (-3.5, 1.5), has_lead=True)
    a = float(LoC.update(True, still, 1.2, False, (-3.5, 1.5), has_lead=True))   # one frame of "go"
    assert LoC.long_control_state == LongCtrlState.stopping and a <= -0.99, (LoC.long_control_state, a)
    for _ in range(5):
      a = float(LoC.update(True, still, 0.0, True, (-3.5, 1.5), has_lead=True))  # stop again
    assert a <= -0.99
    # a sustained go leaves the hold after the debounce
    n = 0
    while LoC.long_control_state == LongCtrlState.stopping and n < 100:
      LoC.update(True, still, 1.2, False, (-3.5, 1.5), has_lead=True); n += 1
    assert abs(n * DT_CTRL - STOPPING_EXIT_DEBOUNCE) < 0.03, n

  def test_request_keeps_easing_with_the_plan_before_the_wheels_stop(self):
    from opendbc.car.structs import car
    CP = car.CarParams.new_message(stopAccel=-1.0)
    LoC = LongControl(CP, custom.CarParamsSP.new_message())
    LoC.long_control_state = LongCtrlState.stopping
    LoC.last_output_accel = -0.40
    rolling = car.CarState.new_message(vEgo=0.2, standstill=False)
    for _ in range(30):
      a = float(LoC.update(True, rolling, -0.20, True, (-3.5, 1.5), has_lead=True))   # plan eases to -0.20
    assert abs(a + 0.20) < 1e-6, a                                                   # followed (lighter)
    a = float(LoC.update(True, rolling, -0.60, True, (-3.5, 1.5), has_lead=True))     # plan firmer: not followed
    assert abs(a + 0.20) < 1e-6, a
    for _ in range(10):
      a = float(LoC.update(True, rolling, 0.0, True, (-3.5, 1.5), has_lead=True))     # plan at 0 (flicker): not followed
    assert abs(a + 0.20) < 1e-6, a
    assert STOPPING_FOLLOW_MIN < 0.0

  def test_every_stop_tapers_in_the_last_metre_whatever_the_planner_asks(self):
    from opendbc.car.structs import car
    from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap import A_1KPH, A_2KPH, A_3KPH
    CP = car.CarParams.new_message(stopAccel=-1.0)
    CP.longitudinalTuning.kiBP = [0.0]
    CP.longitudinalTuning.kiV = [0.0]
    LoC = LongControl(CP, custom.CarParamsSP.new_message())
    LoC.long_control_state = LongCtrlState.pid
    # a hard request at 2 km/h (no lag: aEgo 0) is capped at the 2 km/h curve value, at 1 km/h at the 1 km/h value
    a2 = float(LoC.update(True, car.CarState.new_message(vEgo=2.0 / 3.6, aEgo=0.0), -1.5, False, (-3.5, 1.5)))
    assert abs(a2 + A_2KPH) < 0.02, a2
    for _ in range(100):   # the cap may only lighten the brake at END_TAPER_RELEASE_JERK, so give it time
      LoC.long_control_state = LongCtrlState.pid
      a1 = float(LoC.update(True, car.CarState.new_message(vEgo=1.0 / 3.6, aEgo=0.0), -1.5, False, (-3.5, 1.5)))
    assert abs(a1 + A_1KPH) < 0.02, a1
    # with the car still decelerating, the curve is read ahead (lighter still)
    assert end_taper_decel(2.0 / 3.6, -0.8) < A_2KPH
    # above the taper speed nothing is capped, positive targets and a light target pass unchanged
    assert end_taper_decel(END_TAPER_V_START, 0.0) >= A_3KPH - 1e-9
    LoC.long_control_state = LongCtrlState.pid
    a_up = float(LoC.update(True, car.CarState.new_message(vEgo=1.0 / 3.6, aEgo=0.0), 0.6, False, (-3.5, 1.5)))
    assert a_up > 0.5, a_up

  def test_creep_after_the_stop_adds_brake_gently_then_more(self):
    from opendbc.car.structs import car
    from openpilot.common.realtime import DT_CTRL
    CP = car.CarParams.new_message(stopAccel=-1.0)
    LoC = LongControl(CP, custom.CarParamsSP.new_message())
    LoC.long_control_state = LongCtrlState.stopping
    LoC.last_output_accel = -0.20
    still = car.CarState.new_message(vEgo=0.0, standstill=True)
    creep = car.CarState.new_message(vEgo=0.08, standstill=False)
    for _ in range(10):
      a = float(LoC.update(True, still, -0.1, True, (-3.5, 1.5), has_lead=True))
    assert abs(a + 0.20) < 1e-6, a                              # stopped: frozen
    a0 = a
    for _ in range(int(0.25 / DT_CTRL)):
      a = float(LoC.update(True, creep, -0.1, True, (-3.5, 1.5), has_lead=True))
    first = a0 - a                                              # brake added in the first 0.25 s of creep
    assert 0.0 < first < 0.2, first                             # a little
    for _ in range(int(0.25 / DT_CTRL)):
      a2 = float(LoC.update(True, creep, -0.1, True, (-3.5, 1.5), has_lead=True))
    second = a - a2
    assert second > first * 1.5, (first, second)                # more if it keeps creeping
    for _ in range(5):
      a3 = float(LoC.update(True, still, -0.1, True, (-3.5, 1.5), has_lead=True))
    assert abs(a3 - a2) < 1e-6                                  # stopped again: held where it is
    assert CREEP_RATE_BASE > 0.0
