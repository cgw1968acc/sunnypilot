# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md).
import unittest
from unittest.mock import patch

from opendbc.sunnypilot.car.toyota.auto_brake_hold import (AutoBrakeHoldCarController, BRAKE_HOLD_DECEL, BRAKE_HOLD_LIGHT_TIMER,
                                                            GAS_HOLD_DECEL, GAS_STEP_DECEL, GAS_STEP_FRAMES, HOLD_MAX,
                                                            hold_decel_for_pitch)
from opendbc.sunnypilot.car.toyota.tests.test_auto_brake_hold import FakeCarState, make_car_params, make_car_params_sp


class FakeWheelPulse:
  def __init__(self):
    self.zero_steps = 0


def engage(hybrid: bool, pitch_deg: float = 1.0, force: float = 1000.0, wheel_pulse=None):
  """a light manual stop: the driver holds `force`, then lifts; returns the engaged controller"""
  ctrl = AutoBrakeHoldCarController(make_car_params(hybrid), make_car_params_sp())
  ctrl.update(FakeCarState(standstill=False, brake_pressed=True), 0, None)
  for i in range(1, BRAKE_HOLD_LIGHT_TIMER + 3):
    cs = FakeCarState(brake_pressed=True)
    cs.brake_force = force
    cs.wheel_pulse = wheel_pulse
    ctrl.update(cs, i, None, pitch_deg=pitch_deg)
  cs = FakeCarState(brake_pressed=False)
  cs.wheel_pulse = wheel_pulse
  ctrl.update(cs, BRAKE_HOLD_LIGHT_TIMER + 3, None, pitch_deg=pitch_deg)
  assert ctrl.active
  return ctrl


def hold_frames(ctrl, wheel_pulse, frames, start=1000, creep_every=None, standstill=True):
  """keep the car held; every `creep_every` frames the pulse counter steps forward (wheel speeds still read 0)"""
  reqs = []
  for k in range(frames):
    if creep_every and k % creep_every == 0:
      wheel_pulse.zero_steps += 1
    cs = FakeCarState(standstill=standstill, brake_pressed=False)
    cs.wheel_pulse = wheel_pulse
    ctrl.update(cs, start + k, None)
    reqs.append(ctrl._hold_req)
  return reqs


@patch("opendbc.sunnypilot.car.toyota.auto_brake_hold.toyotacan.create_brake_hold_command")
class TestPetrolAutoBrakeHold(unittest.TestCase):
  def test_hybrid_is_unchanged(self, mock_create):
    wp = FakeWheelPulse()
    ctrl = engage(True, wheel_pulse=wp)
    reqs = hold_frames(ctrl, wp, 300, creep_every=30)   # steps never firm a hybrid's hold
    self.assertEqual(ctrl._hold_decel, BRAKE_HOLD_DECEL)
    self.assertAlmostEqual(reqs[-1], BRAKE_HOLD_DECEL)

  def test_petrol_flat_holds_gas_decel_from_the_first_frame(self, mock_create):
    wp = FakeWheelPulse()
    ctrl = engage(False, wheel_pulse=wp)
    self.assertEqual(ctrl._hold_decel, GAS_HOLD_DECEL)
    self.assertEqual(ctrl._hold_req, GAS_HOLD_DECEL)   # no light start for the creep to win against
    reqs = hold_frames(ctrl, wp, 500)                  # stands still: nothing changes
    self.assertTrue(all(r == GAS_HOLD_DECEL for r in reqs))

  def test_petrol_creep_firms_the_hold_step_by_step_up_to_the_max(self, mock_create):
    wp = FakeWheelPulse()
    ctrl = engage(False, wheel_pulse=wp)
    expected = GAS_HOLD_DECEL
    for _ in range(5):
      wp.zero_steps += 1
      reqs = hold_frames(ctrl, wp, GAS_STEP_FRAMES + 5, standstill=False)
      expected = max(-HOLD_MAX, expected - GAS_STEP_DECEL)
      self.assertAlmostEqual(ctrl._hold_decel, expected)
      self.assertAlmostEqual(reqs[-1], expected)
      steps = [a - b for a, b in zip(reqs, reqs[1:], strict=False)]
      self.assertTrue(all(0.0 <= s <= GAS_STEP_DECEL / GAS_STEP_FRAMES + 1e-9 for s in steps))   # a 0.1 s ramp, no jump
    self.assertAlmostEqual(ctrl._hold_decel, -HOLD_MAX)

  def test_petrol_slope_keeps_the_firmer_grade_schedule(self, mock_create):
    ctrl = engage(False, pitch_deg=-7.0, force=1000.0, wheel_pulse=FakeWheelPulse())
    self.assertAlmostEqual(ctrl._hold_decel, min(hold_decel_for_pitch(-7.0), GAS_HOLD_DECEL))
    self.assertLess(ctrl._hold_decel, GAS_HOLD_DECEL)

  def test_petrol_without_a_pulse_counter_still_holds_firmer(self, mock_create):
    ctrl = engage(False, wheel_pulse=None)
    self.assertEqual(ctrl._hold_decel, GAS_HOLD_DECEL)
    hold_frames(ctrl, FakeWheelPulse(), 0)


if __name__ == "__main__":
  unittest.main()
