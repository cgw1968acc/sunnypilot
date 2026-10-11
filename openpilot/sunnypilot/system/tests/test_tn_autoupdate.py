from opendbc.car.structs import car

import openpilot.sunnypilot.system.tn_autoupdate as M

D, P = car.CarState.GearShifter.drive, car.CarState.GearShifter.park


def test_condition_is_d_with_main_off_and_not_engaged():
  assert M.drive_gear_main_off(D, False, False)
  assert not M.drive_gear_main_off(P, False, False)
  assert not M.drive_gear_main_off(D, True, False)
  assert not M.drive_gear_main_off(D, False, True)


class FakeParams:
  def __init__(self, disable=False):
    self.disable = disable
  def get_bool(self, k):
    return self.disable if k == "DisableUpdates" else False
  def get(self, k):
    return "tnpb3" if k == "UpdaterTargetBranch" else None
  def put_bool(self, k, v):
    pass


class Probe(M.DriveGearUpdater):
  def __init__(self, params, pending=True):
    super().__init__(params)
    self.next_fetch = float("inf")
    self._pending, self.installed = pending, 0
  def pending(self):
    return self._pending
  def install(self):
    self.installed += 1


def test_installs_only_after_the_condition_holds():
  u = Probe(FakeParams())
  assert not u.step(1.0, True) and not u.step(1.0, True)
  assert u.step(1.0, True) and u.installed == 1   # 3 s held


def test_condition_break_resets_the_hold():
  u = Probe(FakeParams())
  u.step(1.0, True); u.step(1.0, True); u.step(1.0, False)
  assert not u.step(1.0, True) and u.installed == 0


def test_nothing_without_a_new_commit_or_with_updates_disabled():
  u = Probe(FakeParams(), pending=False)
  assert not any(u.step(1.0, True) for _ in range(10))
  u = Probe(FakeParams(disable=True))
  assert not any(u.step(1.0, True) for _ in range(10))
