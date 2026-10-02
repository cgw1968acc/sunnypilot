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
# the light stop then waited 2.5 s. Since the standstill flag comes from the wheel pulse counter (the car really
# stands), the waits are no longer needed (driver 2026-10-03, after a slope stop where the hold came 2.5 s late and he
# had to stomp: "with a real wheel-stop signal the delay is unnecessary, never more than 0.2 s"): 0.1 s after a firm
# press, 0.2 s after a light stop.
BRAKE_HOLD_ALLOWED_TIMER = 10  # frames (0.1 s) after the firm press
# Like the factory electronic-parking-brake hold (Corolla Cross), the hold only arms when the driver has pressed the
# pedal firmly at the stop: brake pressure (BRAKE 0xA6 BRAKE_FORCE) must reach BRAKE_HOLD_MIN_FORCE at some point
# during the standstill. Altis 2026-09-30 night: an ordinary light stop sits at 600-1050 N, a deliberate firm press is
# well above. A light stop stays unheld, so nothing is clamped under a driver who is just easing to a halt. When the
# car does not broadcast the force (nan) the hold arms as before.
BRAKE_HOLD_MIN_FORCE = 1400.0  # N
# ... and without a firm press the hold still engages once the car has stood still this long (driver 2026-09-30:
# a long stop at a light should be held too, a short creep-stop should not)
BRAKE_HOLD_LIGHT_TIMER = 20  # frames (0.2 s)
# ... but only if the driver is holding the car with at least this force at that moment (driver 2026-10-02, after the
# 22:30 four-bookmark stop): pressing ~760-800 N the brake only just balanced the hybrid's creep torque, so when the hold
# clamped to its fixed ~1360 N the body rocked (IMU +0.61 / -0.42); at 960 N (21:17:44, 00:13:48) the same clamp was not
# felt. Lighter than this the hold waits (it engages as soon as the driver presses a little more); released lightly, the
# car creeps as it would without the hold. Once engaged the hold stays on whatever the pedal does.
BRAKE_HOLD_LIGHT_MIN_FORCE = 900.0  # N
# Both timers count from the REAL stop (driver 2026-10-01 22:30, four bookmarks: a very slow, very gentle manual stop
# and the hold "suddenly cut in" while the car still rolled below the ~0.5 km/h wheel-speed floor). The standstill flag
# now comes from the wheel pulse counter as well (wheel_pulse.py): while the car still creeps it is not standstill, so
# these timers restart until it really stands.
# Once ENGAGED the hold must not give up at the first wheel pulse. Route 000000ec 2026-10-03 23:51:59 and 23:52:06
# (steep downhill, driver's bookmarks): the driver lifted off, the hold clamped its fixed ~1360 N, 0.4-0.7 s later the
# pulse counter stepped once (the car gave a few cm), standstill went false, the old code dropped the hold at once, the
# PCM released the brake entirely and the car rolled away freely (0 -> 6-11 km/h at ~1.7 m/s^2 with no pedal) until
# the driver caught it. Driver: "I felt the car held, then the slope was too steep, the hold gave up and it slid
# forward". So the hold now stays on while the WHEEL SPEEDS still read zero (the window in which panda lets the 0x344
# override through at all: toyota_tx_hook refuses it once vehicle_moving), whatever the pulse counter says; a creep
# before engagement still restarts the timers. On a slope where 1360 N is less than the grade force the PCM's hold
# cannot physically hold the car - it can only drag - and the driver must keep the pedal; a stronger hold needs ACC
# engaged (stopAccel) or a different PCM channel.

DISALLOWED_GEARS = (GearShifter.park, GearShifter.reverse)

# The hold message is PRE_COLLISION_2 with PBRTRGR set and DSS1GDRV, the PCS deceleration request (signed 10 bit,
# 0.1 m/s^2 per count). The original code wrote the magic 0x3FF, which packs to raw -10 = -1.0 m/s^2; the PCM answers
# that with a fixed ~1360 N (BRAKE 0xA6), not enough on a steep slope (000000ec 23:51:59: rolled away at ~1.7 m/s^2).
# Driver 2026-10-03: "can the hold not ask for more, say 1800 N?" - untested whether the PCM scales the clamp with
# this request; try -1.5 / -2.0 here parked on a flat lot and read 0xA6 before relying on it.
BRAKE_HOLD_DECEL = -1.5  # m/s^2 requested in DSS1GDRV while holding (driver 2026-10-03: try -1.5 first; -1.0 gave ~1360 N)

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
    self._engaged = False

  def update(self, CS: structs.CarState, frame: int, packer) -> list:
    ws = CS.out.wheelSpeeds
    wheels_zero = max(abs(ws.fl), abs(ws.fr), abs(ws.rl), abs(ws.rr)) < 1e-3  # panda's vehicle_moving == false
    relay_blocked = (wheels_zero and CS.out.cruiseState.available and not CS.out.cruiseState.enabled and
                      not CS.out.gasPressed)
    # before engagement the car must really stand (wheel pulse standstill); once engaged the hold persists for as long
    # as the wheel speeds read zero, so one pulse step on a slope does not drop it
    hold_allowed = relay_blocked and CS.out.gearShifter not in DISALLOWED_GEARS and (CS.out.standstill or self._engaged)

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
      held_long = self._counter > BRAKE_HOLD_LIGHT_TIMER and (math.isnan(force) or force >= BRAKE_HOLD_LIGHT_MIN_FORCE)
      if (firm_ready or held_long) and not self._released:
        self._engaged = True
      self.active = self._engaged and not self._released
    else:
      self._counter = 0
      self.active = False
      self._released = False
      self._armed = False
      self._engaged = False

    self._prev_brake_pressed = CS.out.brakePressed

    can_sends = []
    if relay_blocked and frame % 2 == 0:
      override = self.active and not pcs_is_active(CS.pre_collision_2)
      can_sends.append(toyotacan.create_brake_hold_command(packer, frame, CS.pre_collision_2, override,
                                                           hold_decel=BRAKE_HOLD_DECEL))

    return can_sends
