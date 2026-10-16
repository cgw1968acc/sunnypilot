from opendbc.car import structs
from opendbc.sunnypilot.car.toyota.door_unlock import DoorUnlockOnPark, UNLOCK_MSG, LOCK_MSG

GearShifter = structs.CarState.GearShifter


def test_door_unlock_once_on_park():
  d = DoorUnlockOnPark()
  assert d.update(GearShifter.park, 0) == []                 # boot in P: nothing
  assert d.update(GearShifter.drive, 1) == []
  msgs = []
  for f in range(2, 60):
    msgs += d.update(GearShifter.park, f)
  assert len(msgs) == 2 and all(m.address == 0x750 and m.dat == UNLOCK_MSG and m.src == 0 for m in msgs)
  assert d.update(GearShifter.park, 100) == []
  assert d.update(GearShifter.reverse, 101) == [] and len(d.update(GearShifter.park, 102)) == 1


def test_bytes_match_panda_safety():
  # toyota.h: GET_BYTES(msg, 0, 4) == 0x11300540 and GET_BYTES(msg, 4, 4) == 0x00004000 unlock / 0x00008000 lock
  assert int.from_bytes(UNLOCK_MSG[:4], 'little') == 0x11300540
  assert int.from_bytes(UNLOCK_MSG[4:], 'little') == 0x00004000
  assert int.from_bytes(LOCK_MSG[:4], 'little') == 0x11300540
  assert int.from_bytes(LOCK_MSG[4:], 'little') == 0x00008000


def test_door_lock_once_above_10kph_in_drive():
  d = DoorUnlockOnPark()
  assert d.update(GearShifter.park, 0) == []
  assert d.update(GearShifter.drive, 1, 5 / 3.6, False) == []                 # too slow
  assert d.update(GearShifter.drive, 2, 15 / 3.6, True) == []                 # a door open: wait
  assert d.update(GearShifter.reverse, 3, 15 / 3.6, False) == []             # not in D
  msgs = []
  for f in range(4, 60):
    msgs += d.update(GearShifter.drive, f, 15 / 3.6, False)
  assert len(msgs) == 2 and all(m.address == 0x750 and m.dat == LOCK_MSG and m.src == 0 for m in msgs)
  assert all(d.update(GearShifter.drive, f, 30 / 3.6, False) == [] for f in range(60, 200))  # once per drive cycle
  msgs = []
  for f in range(200, 260):                                                   # back to P: unlock, lock re-armed
    msgs += d.update(GearShifter.park, f, 0.0, False)
  assert len(msgs) == 2 and all(m.dat == UNLOCK_MSG for m in msgs)
  msgs = []
  for f in range(260, 320):
    msgs += d.update(GearShifter.drive, f, 12 / 3.6, False)
  assert len(msgs) == 2 and all(m.dat == LOCK_MSG for m in msgs)
