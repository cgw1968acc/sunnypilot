from opendbc.car import structs
from opendbc.sunnypilot.car.toyota.door_unlock import DoorUnlockOnPark, LOCK_SEQ, UNLOCK_SEQ, SEQ_GAP

GearShifter = structs.CarState.GearShifter


def collect(d, gear, frames, v=0.0, door_open=False):
  out = []
  for f in frames:
    for m in d.update(gear, f, v, door_open):
      out.append((f, m))
  return out


def test_door_unlock_once_on_park():
  d = DoorUnlockOnPark()
  assert d.update(GearShifter.park, 0) == []                 # boot in P: nothing
  assert d.update(GearShifter.drive, 1) == []
  msgs = collect(d, GearShifter.park, range(2, 60))
  assert [m.dat for _, m in msgs] == list(UNLOCK_SEQ)
  assert all(m.address == 0x750 and m.src == 0 for _, m in msgs)
  assert [f for f, _ in msgs] == [2 + i * SEQ_GAP for i in range(len(UNLOCK_SEQ))]  # 50 ms apart
  assert d.update(GearShifter.park, 100) == []
  assert d.update(GearShifter.reverse, 101) == []
  assert [m.dat for _, m in collect(d, GearShifter.park, range(102, 140))] == list(UNLOCK_SEQ)


def test_bytes_match_panda_safety():
  # toyota.h sp_valid_uds_msgs: 40 02 10 01|03 (+ zeros) and 40 06 2F 28 | 10 03 0x 80/40
  le = lambda b, i: int.from_bytes(b[i:i + 4], 'little')
  assert [(le(m, 0), le(m, 4)) for m in LOCK_SEQ] == [(0x01100240, 0), (0x03100240, 0), (0x282F0640, 0x80010310), (0x282F0640, 0x80000310)]
  assert [(le(m, 0), le(m, 4)) for m in UNLOCK_SEQ] == [(0x01100240, 0), (0x03100240, 0), (0x282F0640, 0x40010310), (0x282F0640, 0x40000310)]


def test_door_lock_once_above_10kph_in_drive():
  d = DoorUnlockOnPark()
  assert d.update(GearShifter.park, 0) == []
  assert d.update(GearShifter.drive, 1, 5 / 3.6, False) == []                 # too slow
  assert d.update(GearShifter.drive, 2, 15 / 3.6, True) == []                 # a door open: wait
  assert d.update(GearShifter.reverse, 3, 15 / 3.6, False) == []             # not in D
  assert [m.dat for _, m in collect(d, GearShifter.drive, range(4, 60), 15 / 3.6)] == list(LOCK_SEQ)
  assert collect(d, GearShifter.drive, range(60, 200), 30 / 3.6) == []         # once per drive cycle
  assert [m.dat for _, m in collect(d, GearShifter.park, range(200, 260))] == list(UNLOCK_SEQ)  # P: unlock, re-arm
  assert [m.dat for _, m in collect(d, GearShifter.drive, range(260, 320), 12 / 3.6)] == list(LOCK_SEQ)


def test_park_during_lock_sequence_switches_to_unlock():
  d = DoorUnlockOnPark()
  d.update(GearShifter.park, 0)
  first = collect(d, GearShifter.drive, range(1, 3), 15 / 3.6)                 # lock sequence started
  assert [m.dat for _, m in first] == [LOCK_SEQ[0]]
  assert [m.dat for _, m in collect(d, GearShifter.park, range(3, 60))] == list(UNLOCK_SEQ)
