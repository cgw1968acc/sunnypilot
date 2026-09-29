from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import custom
from openpilot.selfdrive.controls.lib.drive_helpers import STOPPING_SPEED, should_stop
from openpilot.selfdrive.controls.lib.longcontrol import STOPPING_DECEL_RATE, STANDSTILL_HOLD_RATE, STANDSTILL_HOLD_DELAY, LongControl, \
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
    """The fast hold ramp starts STANDSTILL_HOLD_DELAY after CS.standstill, not on the flag itself."""
    from opendbc.car.structs import car
    from openpilot.common.realtime import DT_CTRL
    CP = car.CarParams.new_message(stopAccel=-1.0)
    CP_SP = custom.CarParamsSP.new_message()
    LoC = LongControl(CP, CP_SP)
    CS = car.CarState.new_message(vEgo=0.0, aEgo=0.0, standstill=True)
    CS.cruiseState.standstill = False
    LoC.long_control_state = LongCtrlState.stopping
    LoC.last_output_accel = -0.3
    a = [-0.3]
    for _ in range(int(1.0 / DT_CTRL)):
      a.append(float(LoC.update(True, CS, 0.0, True, (-3.5, 1.5))))
    n_delay = int(STANDSTILL_HOLD_DELAY / DT_CTRL)
    early = (a[0] - a[n_delay - 1]) / (STANDSTILL_HOLD_DELAY - DT_CTRL)
    late = (a[n_delay] - a[-1]) / (1.0 - STANDSTILL_HOLD_DELAY)
    assert abs(early - STOPPING_DECEL_RATE) < 0.02, early     # gentle while the car is still settling
    assert abs(late - STANDSTILL_HOLD_RATE) < 0.05, late      # fast once the delay has passed
    assert a[n_delay] > -0.5                                  # the request has barely moved when the wait ends

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
