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
# frames apart, once per shift into P from a driving gear; nothing else ever sends it.
# tncr 2026-10-16: checked against opendbc/safety/modes/toyota.h - the panda DOES pass 40 05 30 11 00 40 00 00 (unlock) and
# 00 80 (lock) on 0x750 (sp_valid_uds_msgs). The commit that ported this file said otherwise; that note was wrong.
UNLOCK_MSG = b'\x40\x05\x30\x11\x00\x40\x00\x00'
UNLOCK_REPEAT = 2
UNLOCK_GAP = 10  # frames (0.1 s)


class DoorUnlockOnPark:
  def __init__(self):
    self.gear_prev = GearShifter.park
    self.left = 0
    self.next_frame = 0

  def update(self, gear: GearShifter, frame: int) -> list[CanData]:
    if gear == GearShifter.park and self.gear_prev not in (GearShifter.park, GearShifter.unknown):
      self.left, self.next_frame = UNLOCK_REPEAT, frame
    self.gear_prev = gear
    if self.left > 0 and frame >= self.next_frame:
      self.left -= 1
      self.next_frame = frame + UNLOCK_GAP
      return [CanData(0x750, UNLOCK_MSG, 0)]
    return []
