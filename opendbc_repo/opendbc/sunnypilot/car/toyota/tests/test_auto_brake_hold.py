import unittest
from unittest.mock import patch

from opendbc.car import structs
from opendbc.sunnypilot.car.toyota.auto_brake_hold import (AutoBrakeHoldCarController, BRAKE_HOLD_ALLOWED_TIMER, BRAKE_HOLD_MIN_FORCE,
                                                            BRAKE_HOLD_LIGHT_TIMER, pcs_is_active)
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

GearShifter = structs.CarState.GearShifter


def make_car_params_sp(enabled: bool = True) -> structs.CarParamsSP:
  cp_sp = structs.CarParamsSP()
  cp_sp.flags = ToyotaFlagsSP.SP_AUTO_BRAKE_HOLD if enabled else 0
  return cp_sp


class FakeCarState:
  def __init__(self, standstill=True, cruise_enabled=False, cruise_available=True, gas_pressed=False,
               gear=GearShifter.drive, brake_pressed=False, pre_collision_2=None, wheel_speed=0.0):
    self.out = structs.CarState()
    self.out.standstill = standstill
    self.out.wheelSpeeds.fl = self.out.wheelSpeeds.fr = self.out.wheelSpeeds.rl = self.out.wheelSpeeds.rr = wheel_speed
    self.out.cruiseState.enabled = cruise_enabled
    self.out.cruiseState.available = cruise_available
    self.out.gasPressed = gas_pressed
    self.out.gearShifter = gear
    self.out.brakePressed = brake_pressed
    self.pre_collision_2 = pre_collision_2 if pre_collision_2 is not None else {}


@patch("opendbc.sunnypilot.car.toyota.auto_brake_hold.toyotacan.create_brake_hold_command")
class TestAutoBrakeHoldCarController(unittest.TestCase):
  def _make(self):
    return AutoBrakeHoldCarController(structs.CarParams(), make_car_params_sp())

  def test_enabled_reflects_flag(self, mock_create):
    for value in range(256):
      with self.subTest(flags=value):
        cp_sp = structs.CarParamsSP()
        cp_sp.flags = value
        ctrl = AutoBrakeHoldCarController(structs.CarParams(), cp_sp)
        self.assertEqual(ctrl.enabled, bool(value & ToyotaFlagsSP.SP_AUTO_BRAKE_HOLD))

  def test_does_not_engage_before_timer(self, mock_create):
    ctrl = self._make()
    cs = FakeCarState(brake_pressed=False)
    for i in range(BRAKE_HOLD_ALLOWED_TIMER):
      ctrl.update(cs, i, None)
      self.assertFalse(ctrl.active)

  def test_engages_after_timer_once_brake_released(self, mock_create):
    ctrl = self._make()
    cs = FakeCarState(brake_pressed=False)
    for i in range(BRAKE_HOLD_ALLOWED_TIMER + 1):
      ctrl.update(cs, i, None)
    self.assertTrue(ctrl.active)

  def test_brake_still_down_from_the_stop_does_not_block_engagement(self, mock_create):
    # the driver's foot is normally still on the brake on the very frame standstill is reached -
    # that must not count as a "fresh press" release, or the feature could never engage
    ctrl = self._make()
    # decelerating into the stop with the brake held continuously
    cs = FakeCarState(standstill=False, brake_pressed=True)
    for i in range(30):
      ctrl.update(cs, i, None)
    # reaches standstill, foot stays down for a while, then lifts
    cs.out.standstill = True
    for i in range(30, 30 + BRAKE_HOLD_ALLOWED_TIMER - 2):
      ctrl.update(cs, i, None)
      self.assertFalse(ctrl.active, "must not engage while the original stopping press is still held")
    cs.out.brakePressed = False
    for i in range(30 + BRAKE_HOLD_ALLOWED_TIMER - 2, 50 + BRAKE_HOLD_ALLOWED_TIMER + 1):
      ctrl.update(cs, i, None)
    self.assertTrue(ctrl.active, "should engage once the stopping press is released and the timer elapses")

  def test_fresh_brake_press_mid_hold_releases_and_stays_released(self, mock_create):
    ctrl = self._make()
    cs = FakeCarState(brake_pressed=False)
    frame = 0
    for _ in range(BRAKE_HOLD_ALLOWED_TIMER + 1):
      ctrl.update(cs, frame, None)
      frame += 1
    self.assertTrue(ctrl.active)

    cs.out.brakePressed = True
    ctrl.update(cs, frame, None)
    frame += 1
    self.assertFalse(ctrl.active, "a fresh press should release immediately")

    for _ in range(20):
      ctrl.update(cs, frame, None)
      frame += 1
      self.assertFalse(ctrl.active, "must stay released for the rest of the episode while continuously held")

    cs.out.brakePressed = False
    for _ in range(20):
      ctrl.update(cs, frame, None)
      frame += 1
      self.assertFalse(ctrl.active, "must stay released for the rest of the episode even after lifting off again")

  def test_drive_off_and_restop_rearms(self, mock_create):
    ctrl = self._make()
    cs = FakeCarState(brake_pressed=False)
    frame = 0
    for _ in range(BRAKE_HOLD_ALLOWED_TIMER + 1):
      ctrl.update(cs, frame, None)
      frame += 1
    cs.out.brakePressed = True
    ctrl.update(cs, frame, None)
    frame += 1
    self.assertFalse(ctrl.active)

    # drives off - leaves the standstill episode entirely (the wheels turn)
    cs = FakeCarState(standstill=False, brake_pressed=True, wheel_speed=2.0)
    ctrl.update(cs, frame, None)
    frame += 1

    # stops again
    cs = FakeCarState(brake_pressed=False)
    for _ in range(BRAKE_HOLD_ALLOWED_TIMER + 1):
      ctrl.update(cs, frame, None)
      frame += 1
    self.assertTrue(ctrl.active, "should re-arm and engage again at the next stop")

  def test_cruise_engaged_blocks_hold(self, mock_create):
    ctrl = self._make()
    cs = FakeCarState(cruise_enabled=True, brake_pressed=False)
    for i in range(BRAKE_HOLD_ALLOWED_TIMER + 1):
      ctrl.update(cs, i, None)
    self.assertFalse(ctrl.active, "must not hold while ACC is engaged - that's the point of the constraint")

  def test_gas_pressed_blocks_hold(self, mock_create):
    ctrl = self._make()
    cs = FakeCarState(gas_pressed=True, brake_pressed=False)
    for i in range(BRAKE_HOLD_ALLOWED_TIMER + 1):
      ctrl.update(cs, i, None)
    self.assertFalse(ctrl.active)

  def test_park_and_reverse_block_hold(self, mock_create):
    for gear in (GearShifter.park, GearShifter.reverse):
      with self.subTest(gear=gear):
        ctrl = self._make()
        cs = FakeCarState(gear=gear, brake_pressed=False)
        for i in range(BRAKE_HOLD_ALLOWED_TIMER + 1):
          ctrl.update(cs, i, None)
        self.assertFalse(ctrl.active)

  def test_yields_to_live_pcs_without_dropping_active_state(self, mock_create):
    ctrl = self._make()
    cs = FakeCarState(brake_pressed=False)
    frame = 0
    for _ in range(BRAKE_HOLD_ALLOWED_TIMER + 1):
      ctrl.update(cs, frame, None)
      frame += 1
    self.assertTrue(ctrl.active)
    mock_create.reset_mock()

    # a real PCS event shows up on the live signal
    cs.pre_collision_2 = {"PCSALM": 1}
    # advance to the next even frame the message is actually built on
    while frame % 2 != 0:
      frame += 1
    ctrl.update(cs, frame, None)
    override_arg = mock_create.call_args.args[-1]
    self.assertTrue(ctrl.active, "internal hold state should not be cleared by a live PCS event")
    self.assertFalse(override_arg, "must not override PRE_COLLISION_2 while PCS is genuinely active")

    # once PCS goes quiet again, override resumes on our own signal, not stale PCS state
    frame += 2
    cs.pre_collision_2 = {}
    ctrl.update(cs, frame, None)
    override_arg = mock_create.call_args.args[-1]
    self.assertTrue(override_arg)

  def test_message_only_built_every_other_frame(self, mock_create):
    ctrl = self._make()
    cs = FakeCarState(brake_pressed=False)
    for i in range(10):
      ctrl.update(cs, i, None)
    self.assertEqual(mock_create.call_count, 5)

  def test_no_message_sent_outside_relay_blocked_window(self, mock_create):
    # outside relay_blocked, toyota_fwd_hook lets the real PRE_COLLISION_2 relay through on its own -
    # our passthrough copy would just be redundant traffic panda rejects, so it must not be sent at all
    cases = [
      dict(cruise_enabled=True),
      dict(gas_pressed=True),
      dict(cruise_available=False),
    ]
    for kwargs in cases:
      with self.subTest(kwargs=kwargs):
        ctrl = self._make()
        cs = FakeCarState(brake_pressed=False, **kwargs)
        for i in range(10):
          ctrl.update(cs, i, None)
        mock_create.assert_not_called()

  def test_message_still_sent_in_park_or_reverse(self, mock_create):
    # relay_blocked has no gear check, matching toyota_fwd_hook - park/reverse must not open a gap
    # in the PRE_COLLISION_2 relay even though hold can never engage there (this was a real
    # regression: gating the send on hold_allowed's gear check left bus 0 with nothing at this
    # address while parked, which is what tripped a genuine PCS dash fault on-road)
    for gear in (GearShifter.park, GearShifter.reverse):
      with self.subTest(gear=gear):
        ctrl = self._make()
        cs = FakeCarState(gear=gear, brake_pressed=False)
        for i in range(BRAKE_HOLD_ALLOWED_TIMER + 1):
          ctrl.update(cs, i, None)
        self.assertFalse(ctrl.active, "must never actually hold in park/reverse")
        self.assertGreater(mock_create.call_count, 0, "must still relay PRE_COLLISION_2 in park/reverse")
        override_arg = mock_create.call_args.args[-1]
        self.assertFalse(override_arg, "never override while gear disallows an actual hold")


class TestPcsIsActive(unittest.TestCase):
  def test_all_zero_is_not_active(self):
    self.assertFalse(pcs_is_active({}))
    self.assertFalse(pcs_is_active({"PCSALM": 0, "DSS1GDRV": 0}))

  def test_any_trigger_field_is_active(self):
    for field in ("PCSALM", "IBTRGR", "PBATRGR", "PREFILL", "AVSTRGR", "PBRTRGR", "PPTRGR"):
      with self.subTest(field=field):
        self.assertTrue(pcs_is_active({field: 1}))

  def test_nonzero_force_signal_is_active(self):
    self.assertTrue(pcs_is_active({"DSS1GDRV": -5}))
    self.assertFalse(pcs_is_active({"DSS1GDRV": 0}))


if __name__ == "__main__":
  unittest.main()


@patch("opendbc.sunnypilot.car.toyota.auto_brake_hold.toyotacan.create_brake_hold_command")
class TestAutoBrakeHoldFirmPress(unittest.TestCase):
  def _run(self, forces):
    ctrl = AutoBrakeHoldCarController(structs.CarParams(), make_car_params_sp())
    ctrl.update(FakeCarState(standstill=False, brake_pressed=True), 0, None)   # the press starts while still rolling
    for i, f in enumerate(forces, 1):
      cs = FakeCarState(brake_pressed=True)
      cs.brake_force = f
      ctrl.update(cs, i, None)
    return ctrl

  def test_light_stop_is_held_only_after_the_longer_wait(self, mock_create):
    self.assertFalse(self._run([BRAKE_HOLD_MIN_FORCE * 0.7] * BRAKE_HOLD_LIGHT_TIMER).active)
    self.assertTrue(self._run([BRAKE_HOLD_MIN_FORCE * 0.7] * (BRAKE_HOLD_LIGHT_TIMER + 1)).active)

  def test_firm_press_engages_one_timer_after_the_press(self, mock_create):
    light, firm = BRAKE_HOLD_MIN_FORCE * 0.6, BRAKE_HOLD_MIN_FORCE * 1.1
    # stopped lightly, then pressed firmly before the light timer: engages BRAKE_HOLD_ALLOWED_TIMER after the press, not before
    self.assertFalse(self._run([light] * 5 + [firm] * BRAKE_HOLD_ALLOWED_TIMER).active)
    self.assertTrue(self._run([light] * 5 + [firm] * (BRAKE_HOLD_ALLOWED_TIMER + 1)).active)
    self.assertLess(BRAKE_HOLD_ALLOWED_TIMER, BRAKE_HOLD_LIGHT_TIMER)

  def test_firm_press_arms_even_if_eased_afterwards(self, mock_create):
    forces = [BRAKE_HOLD_MIN_FORCE * 0.6] * 5 + [BRAKE_HOLD_MIN_FORCE * 1.1] * 2 + [BRAKE_HOLD_MIN_FORCE * 0.6] * (BRAKE_HOLD_ALLOWED_TIMER)
    self.assertTrue(self._run(forces).active)

  def test_missing_force_signal_keeps_the_old_behaviour(self, mock_create):
    self.assertTrue(self._run([float("nan")] * (BRAKE_HOLD_ALLOWED_TIMER + 5)).active)


@patch("opendbc.sunnypilot.car.toyota.auto_brake_hold.toyotacan.create_brake_hold_command")
class TestAutoBrakeHoldRealStop(unittest.TestCase):
  """2026-10-01 22:30: a creep below the wheel-speed floor is no longer standstill (wheel_pulse.py), so the timers
  restart until the car really stands."""
  def test_creep_restarts_the_timers(self, mock_create):
    ctrl = AutoBrakeHoldCarController(structs.CarParams(), make_car_params_sp())
    light = BRAKE_HOLD_MIN_FORCE * 0.7
    for i in range(500):
      creeping = (i % 25) < 10          # every 0.25 s the pulse counter shows motion for a moment
      cs = FakeCarState(standstill=not creeping, brake_pressed=True)
      cs.brake_force = light
      ctrl.update(cs, i, None)
      self.assertFalse(ctrl.active)
    for i in range(BRAKE_HOLD_LIGHT_TIMER + 1):
      cs = FakeCarState(brake_pressed=True)
      cs.brake_force = light
      ctrl.update(cs, 500 + i, None)
    self.assertTrue(ctrl.active)


@patch("opendbc.sunnypilot.car.toyota.auto_brake_hold.toyotacan.create_brake_hold_command")
class TestAutoBrakeHoldLightMinForce(unittest.TestCase):
  """2026-10-02: the 2.5 s light-stop path needs >= BRAKE_HOLD_LIGHT_MIN_FORCE at that moment, and latches."""
  def _ctrl(self):
    ctrl = AutoBrakeHoldCarController(structs.CarParams(), make_car_params_sp())
    ctrl.update(FakeCarState(standstill=False, brake_pressed=True), 0, None)
    return ctrl

  def _step(self, ctrl, i, force, pressed=True):
    cs = FakeCarState(brake_pressed=pressed)
    cs.brake_force = force
    ctrl.update(cs, i, None)
    return ctrl.active

  def test_very_light_hold_waits(self, mock_create):
    from opendbc.sunnypilot.car.toyota.auto_brake_hold import BRAKE_HOLD_LIGHT_MIN_FORCE
    ctrl = self._ctrl()
    for i in range(1, 600):
      self.assertFalse(self._step(ctrl, i, BRAKE_HOLD_LIGHT_MIN_FORCE - 100))
    self.assertTrue(self._step(ctrl, 600, BRAKE_HOLD_LIGHT_MIN_FORCE + 50))   # presses a little more: engages

  def test_engaged_hold_survives_a_wheel_pulse_while_the_wheels_read_zero(self, mock_create):
    # 2026-10-03 slope roll-away: one pulse step after engagement must not drop the hold
    ctrl = self._ctrl()
    for i in range(1, BRAKE_HOLD_LIGHT_TIMER + 2):
      self._step(ctrl, i, 960.0)
    self.assertTrue(ctrl.active)
    for i in range(BRAKE_HOLD_LIGHT_TIMER + 2, BRAKE_HOLD_LIGHT_TIMER + 60):
      cs = FakeCarState(standstill=False, brake_pressed=False)   # pulse counter moved, wheel speeds still 0
      ctrl.update(cs, i, None)
      self.assertTrue(ctrl.active)
    cs = FakeCarState(standstill=False, brake_pressed=False, wheel_speed=0.3)   # the wheels turn: panda would refuse 0x344
    ctrl.update(cs, 1000, None)
    self.assertFalse(ctrl.active)

  def test_released_hold_does_not_carry_over_a_creep_to_the_next_stop(self, mock_create):
    # route 000000ee 11:55: hold, fresh press releases it, creep below the wheel-speed floor, new stop must hold again
    ctrl = self._ctrl()
    for i in range(1, BRAKE_HOLD_LIGHT_TIMER + 2):
      self._step(ctrl, i, 1120.0)
    self.assertTrue(ctrl.active)
    f = BRAKE_HOLD_LIGHT_TIMER + 2
    self._step(ctrl, f, 2040.0, pressed=False)                        # foot off, held
    f += 1
    for _ in range(10):                                                # fresh press: released
      self.assertFalse(self._step(ctrl, f, 450.0, pressed=True))
      f += 1
    for i in range(20):                                                # creeping, wheel speeds still zero; braking again at the end
      cs = FakeCarState(standstill=False, brake_pressed=i >= 15)
      cs.brake_force = 800.0 if i >= 15 else 200.0
      ctrl.update(cs, f, None)
      f += 1
      self.assertFalse(ctrl.active)
    for _ in range(BRAKE_HOLD_LIGHT_TIMER + 2):                        # new stop at 1560 N
      self._step(ctrl, f, 1560.0)
      f += 1
    self.assertTrue(ctrl.active, "the next stop must be held again")

  def test_engaged_hold_latches_when_the_pedal_lightens(self, mock_create):
    ctrl = self._ctrl()
    for i in range(1, BRAKE_HOLD_LIGHT_TIMER + 2):
      self._step(ctrl, i, 960.0)
    self.assertTrue(ctrl.active)
    for i in range(BRAKE_HOLD_LIGHT_TIMER + 2, BRAKE_HOLD_LIGHT_TIMER + 50):
      self.assertTrue(self._step(ctrl, i, 200.0))   # the foot lifts: still held


@patch("opendbc.sunnypilot.car.toyota.auto_brake_hold.toyotacan.create_brake_hold_command")
class TestAutoBrakeHoldSlope(unittest.TestCase):
  """2026-10-05: the old -1.0 request on the flat, -1.5 on a slope, decided once when the hold engages."""
  def _engage(self, pitch_deg):
    from opendbc.sunnypilot.car.toyota.auto_brake_hold import BRAKE_HOLD_LIGHT_TIMER
    ctrl = AutoBrakeHoldCarController(structs.CarParams(), make_car_params_sp())
    ctrl.update(FakeCarState(standstill=False, brake_pressed=True), 0, None)
    for i in range(1, BRAKE_HOLD_LIGHT_TIMER + 3):
      cs = FakeCarState(brake_pressed=True)
      cs.brake_force = 1000.0
      ctrl.update(cs, i, None, pitch_deg=pitch_deg)
    self.assertTrue(ctrl.active)
    return ctrl

  def test_flat_keeps_the_old_request_and_a_slope_gets_the_firmer_one(self, mock_create):
    from opendbc.sunnypilot.car.toyota.auto_brake_hold import BRAKE_HOLD_DECEL, BRAKE_HOLD_DECEL_SLOPE
    self.assertEqual(self._engage(1.0)._hold_decel, BRAKE_HOLD_DECEL)
    self.assertEqual(self._engage(1.9)._hold_decel, BRAKE_HOLD_DECEL)
    self.assertEqual(self._engage(-7.0)._hold_decel, BRAKE_HOLD_DECEL_SLOPE)   # the 2026-10-03 hill
    self.assertEqual(self._engage(3.5)._hold_decel, BRAKE_HOLD_DECEL_SLOPE)    # uphill driveway
    self.assertEqual(self._engage(float("nan"))._hold_decel, BRAKE_HOLD_DECEL)

  def test_the_choice_is_latched_while_held(self, mock_create):
    from opendbc.sunnypilot.car.toyota.auto_brake_hold import BRAKE_HOLD_DECEL_SLOPE
    ctrl = self._engage(-7.0)
    for i in range(500, 540):
      cs = FakeCarState(brake_pressed=False)
      cs.brake_force = 2000.0
      ctrl.update(cs, i, None, pitch_deg=1.0)   # pitch reads flat once the body settles: still the slope hold
    self.assertTrue(ctrl.active)
    self.assertEqual(ctrl._hold_decel, BRAKE_HOLD_DECEL_SLOPE)
    self.assertEqual(mock_create.call_args.kwargs.get("hold_decel"), BRAKE_HOLD_DECEL_SLOPE)
