# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from opendbc.car import structs
from opendbc.sunnypilot.car.toyota.door_unlock import DoorLock, LOCK_SEQ, UNLOCK_SEQ, SEQ_GAP

GearShifter = structs.CarState.GearShifter


def collect(d, gear, frames, v=0.0, door_open=False):
  out = []
  for f in frames:
    for m in d.update(gear, f, v, door_open):
      out.append((f, m))
  return out


def le(b, i):
  return int.from_bytes(b[i:i + 4], 'little')


def test_bytes_match_panda_safety():
  # toyota.h sp_valid_uds_msgs: 40 02 10 01|03 (+ zeros) and 40 06 2F 28 | 10|11 03 0x 80
  assert [(le(m, 0), le(m, 4)) for m in LOCK_SEQ] == [(0x01100240, 0), (0x03100240, 0), (0x282F0640, 0x80010310), (0x282F0640, 0x80000310)]
  assert [(le(m, 0), le(m, 4)) for m in UNLOCK_SEQ] == [(0x01100240, 0), (0x03100240, 0), (0x282F0640, 0x80010311), (0x282F0640, 0x80000311)]


def test_door_lock_once_above_10kph_in_drive():
  d = DoorLock()
  assert d.update(GearShifter.park, 0) == []
  assert d.update(GearShifter.drive, 1, 5 / 3.6, False) == []                 # too slow
  assert d.update(GearShifter.drive, 2, 15 / 3.6, True) == []                 # a door open: wait
  assert d.update(GearShifter.reverse, 3, 15 / 3.6, False) == []             # not in D
  msgs = collect(d, GearShifter.drive, range(4, 60), 15 / 3.6)
  assert [m.dat for _, m in msgs] == list(LOCK_SEQ)
  assert all(m.address == 0x750 and m.src == 0 for _, m in msgs)
  assert [f for f, _ in msgs] == [4 + i * SEQ_GAP for i in range(len(LOCK_SEQ))]  # 50 ms apart
  assert collect(d, GearShifter.drive, range(60, 200), 30 / 3.6) == []         # once per drive cycle


def test_park_sends_nothing_and_rearms():
  d = DoorLock()
  d.update(GearShifter.park, 0)
  collect(d, GearShifter.drive, range(1, 60), 15 / 3.6)
  assert collect(d, GearShifter.park, range(60, 120)) == []                    # no unlock on P (pandad, ignition off)
  assert [m.dat for _, m in collect(d, GearShifter.drive, range(120, 180), 12 / 3.6)] == list(LOCK_SEQ)


def test_park_during_lock_sequence_finishes_the_lock():
  d = DoorLock()
  d.update(GearShifter.park, 0)
  first = collect(d, GearShifter.drive, range(1, 3), 15 / 3.6)
  assert [m.dat for _, m in first] == [LOCK_SEQ[0]]
  assert [m.dat for _, m in collect(d, GearShifter.park, range(3, 60))] == list(LOCK_SEQ[1:])
