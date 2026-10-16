"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.car.can_definitions import CanData

GearShifter = structs.CarState.GearShifter

# Unlock all doors when shifting into P (owner 2026-10-14: "unlock all four doors when parking"). The body ECU takes it
# on the diagnostic address 0x750, sub-address 0x40: 40 05 30 11 00 40 00 00 = unlock (00 80 = lock), the pair the
# panda Toyota safety already allows ("automatic door locking and unlocking"). Sent UNLOCK_REPEAT times, UNLOCK_GAP
# frames apart, once per shift into P from a driving gear.
# Lock (owner 2026-10-17: "lock automatically once driving above 10 km/h"): the same frame with 00 80 (the pair the
# panda allows, opendbc/safety/modes/toyota.h sp_valid_uds_msgs 0x00004000 unlock / 0x00008000 lock), sent once per
# drive cycle: in D, above LOCK_V with all doors closed, after the car has been in P (the next P unlocks again and
# re-arms). Route 00000160 (10-17) showed the unlock frames go out and are echoed by the panda on bus 0.
UNLOCK_MSG = b'\x40\x05\x30\x11\x00\x40\x00\x00'
LOCK_MSG = b'\x40\x05\x30\x11\x00\x80\x00\x00'
UNLOCK_REPEAT = 2
UNLOCK_GAP = 10  # frames (0.1 s)
LOCK_V = 10.0 / 3.6  # m/s


class DoorUnlockOnPark:
  def __init__(self):
    self.gear_prev = GearShifter.park
    self.left = 0
    self.next_frame = 0
    self.msg = UNLOCK_MSG
    self.lock_armed = True  # boot in P: the first drive above LOCK_V locks

  def update(self, gear: GearShifter, frame: int, v_ego: float = 0.0, door_open: bool = False) -> list[CanData]:
    if gear == GearShifter.park and self.gear_prev not in (GearShifter.park, GearShifter.unknown):
      self.left, self.next_frame, self.msg = UNLOCK_REPEAT, frame, UNLOCK_MSG
      self.lock_armed = True
    elif self.lock_armed and gear == GearShifter.drive and v_ego > LOCK_V and not door_open and self.left == 0:
      self.left, self.next_frame, self.msg = UNLOCK_REPEAT, frame, LOCK_MSG
      self.lock_armed = False
    self.gear_prev = gear
    if self.left > 0 and frame >= self.next_frame:
      self.left -= 1
      self.next_frame = frame + UNLOCK_GAP
      return [CanData(0x750, self.msg, 0)]
    return []
