# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from types import SimpleNamespace as NS

from opendbc.car import structs
from opendbc.sunnypilot.car.toyota.engage_close_stop import EngageCloseStop, b_need
from opendbc.sunnypilot.car.toyota.door_unlock import DoorUnlockOnPark, UNLOCK_MSG, LOCK_MSG
from opendbc.sunnypilot.car.toyota.end_force import end_force_cap
from opendbc.sunnypilot.car.toyota.brake_onset import BrakeOnsetShaper

GearShifter = structs.CarState.GearShifter


def lead(d, v=0.0, status=True):
  return NS(dRel=d, vLead=v, status=status)


def test_close_engage_is_detected_far_is_not():
  c = EngageCloseStop()
  c.update(lead(4.2), 7.2 / 3.6, False, -1.2, 0.0)  # route 0000015a 18:51:24
  assert c.active
  c = EngageCloseStop()
  c.update(lead(10.0), 9.6 / 3.6, False, -0.9, 0.0)  # route 00000159 08:54:34: keeps the soft start
  assert not c.active
  c = EngageCloseStop()
  c.update(lead(4.2), 7.2 / 3.6, False, -1.2, None)  # not in the engage window
  assert not c.active


def test_cap_is_what_the_stop_needs_and_ends_at_standstill():
  c = EngageCloseStop()
  c.update(lead(4.2), 2.0, False, -1.23, 0.0)
  cap = c.cap(-1.23, lead(4.2), 2.0)
  assert abs(cap - -(b_need(2.0, 4.2) * 1.15 + 0.1)) < 1e-9 and cap > -1.23
  assert c.cap(-0.4, lead(4.2), 2.0) == -0.4               # lighter requests pass
  c.update(lead(2.0), 0.0, True, -0.3, 1.0)
  assert not c.active


def test_close_engage_builds_linearly_from_the_first_frame():
  c = EngageCloseStop()
  dt = 0.03
  s, s_close = BrakeOnsetShaper(dt, 4.0 / dt), BrakeOnsetShaper(dt, 4.0 / dt)
  a, a_close, t = 0.0, 0.0, 0.0
  while t < 0.5 - 1e-9:
    a += s.down_step(-1.2, a, v_ego=2.0, t_engaged=t)
    a_close += s_close.down_step(-1.2, a_close, v_ego=2.0, t_engaged=t, engage_j_down=c.j_down)
    t += dt
  assert a >= -0.26                    # far SET-: still the -0.25 soft start
  assert a_close <= -0.45              # close stop: ~1.0 m/s^3 from the first frame (-0.5 at 0.5 s)


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


def test_unlock_bytes_match_panda_safety():
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


def test_end_force_cap_below_2kph_only():
  assert end_force_cap(-0.48, -0.30, 1.0 / 3.6, False) == -0.30   # sent command never firmer than the request
  assert end_force_cap(-1.31, -1.09, 4.8 / 3.6, False) == -1.31   # above 2 km/h untouched
  assert end_force_cap(-1.5, -1.2, 0.0, True) == -1.5             # standstill hold untouched
  assert end_force_cap(-0.2, -0.3, 1.0 / 3.6, False) == -0.2
