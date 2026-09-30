"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import math

from opendbc.car import structs
from opendbc.car.toyota import toyotacan
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

GearShifter = structs.CarState.GearShifter

# Two ways to engage (driver 2026-09-30 night, modelled on the factory EPB hold of the Corolla Cross):
#  - firm press at the stop: BRAKE_HOLD_ALLOWED_TIMER after the moment the brake force first reaches
#    BRAKE_HOLD_MIN_FORCE while standing still;
#  - no firm press: BRAKE_HOLD_LIGHT_TIMER after the wheels stopped.
# The hold's pre-brake clamp is a step (PBRTRGR, not a ramped request). rlog 000000dc-e0: with the old fixed 1 s after
# the standstill flag it showed as a -0.4..-0.6 m/s^2 jolt on 2 of 8 light manual stops, felt as part of stopping;
# a light stop now waits 2.5 s, and after a deliberate firm press the clamp is expected.
BRAKE_HOLD_ALLOWED_TIMER = 100  # frames (1 s) after the firm press
# Like the factory electronic-parking-brake hold (Corolla Cross), the hold only arms when the driver has pressed the
# pedal firmly at the stop: brake pressure (BRAKE 0xA6 BRAKE_FORCE) must reach BRAKE_HOLD_MIN_FORCE at some point
# during the standstill. Altis 2026-09-30 night: an ordinary light stop sits at 600-1050 N, a deliberate firm press is
# well above. A light stop stays unheld, so nothing is clamped under a driver who is just easing to a halt. When the
# car does not broadcast the force (nan) the hold arms as before.
BRAKE_HOLD_MIN_FORCE = 1400.0  # N
# ... and without a firm press the hold still engages once the car has stood still this long (driver 2026-09-30:
# a long stop at a light should be held too, a short creep-stop should not)
BRAKE_HOLD_LIGHT_TIMER = 250  # frames (2.5 s)

DISALLOWED_GEARS = (GearShifter.park, GearShifter.reverse)

# PRE_COLLISION_2 fields that go high when the camera's own PCS/AEB is genuinely intervening this
# frame (PCSALM mirrors PRECOLLISION_ACTIVE; IBTRGR/PBATRGR/PREFILL/AVSTRGR/PBRTRGR/PPTRGR are its
# actuation triggers - see create_pcs_commands for the same field set on the stock-DSU PCS path).
# Deliberately over-inclusive: a false positive here just means we pass a quiescent frame through
# instead of holding it, never the other way around, so err toward checking more fields, not fewer.
PCS_TRIGGER_FIELDS = ("PCSALM", "IBTRGR", "PBATRGR", "PREFILL", "AVSTRGR", "PBRTRGR", "PPTRGR")


def pcs_is_active(pre_collision_2: dict) -> bool:
  return any(pre_collision_2.get(field, 0) for field in PCS_TRIGGER_FIELDS) or pre_collision_2.get("DSS1GDRV", 0) != 0


class AutoBrakeHold:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

  @property
  def enabled(self):
    return bool(self.CP_SP.flags & ToyotaFlagsSP.SP_AUTO_BRAKE_HOLD)


# Auto Brake Hold (@AlexandreSato, @rav4kumar): holds the car at a stop with cruise off by
# overriding PRE_COLLISION_2 - the only channel on this platform that can command the brake
# independent of ACC engagement, since PCS/AEB is an always-on active safety system by design.
# Yields to any genuine PCS activation this frame - the real message is only ever overridden while
# it's quiescent - and releases for the rest of the current standstill episode on a brake press,
# rather than for a single frame.
class AutoBrakeHoldCarController(AutoBrakeHold):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)

    self.active = False
    self._counter = 0
    self._released = False
    self._armed = False
    self._firm_frame = 0
    self._prev_brake_pressed = False

  def update(self, CS: structs.CarState, frame: int, packer) -> list:
    relay_blocked = (CS.out.standstill and CS.out.cruiseState.available and not CS.out.cruiseState.enabled and
                      not CS.out.gasPressed)
    hold_allowed = relay_blocked and CS.out.gearShifter not in DISALLOWED_GEARS

    if hold_allowed:
      # only a fresh press releases hold - the press that caused the stop is already reflected in
      # _prev_brake_pressed by the time standstill is reached, so it doesn't count as a release
      if CS.out.brakePressed and not self._prev_brake_pressed:
        self._released = True
      force = getattr(CS, "brake_force", float("nan"))
      self._counter += 1
      if not self._armed and (math.isnan(force) or force >= BRAKE_HOLD_MIN_FORCE):
        self._armed = True
        self._firm_frame = self._counter
      firm_ready = self._armed and self._counter - self._firm_frame >= BRAKE_HOLD_ALLOWED_TIMER
      held_long = self._counter > BRAKE_HOLD_LIGHT_TIMER
      self.active = (firm_ready or held_long) and not self._released
    else:
      self._counter = 0
      self.active = False
      self._released = False
      self._armed = False

    self._prev_brake_pressed = CS.out.brakePressed

    can_sends = []
    if relay_blocked and frame % 2 == 0:
      override = self.active and not pcs_is_active(CS.pre_collision_2)
      can_sends.append(toyotacan.create_brake_hold_command(packer, frame, CS.pre_collision_2, override))

    return can_sends
