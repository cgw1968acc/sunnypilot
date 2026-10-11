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


M.enabled = lambda: True  # the owner's device; test_off_without_the_flag checks the default


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


def test_off_without_the_flag(monkeypatch=None):
  saved = M.enabled
  try:
    M.enabled = lambda: False  # any other user's device: no flag file
    u = Probe(FakeParams())
    assert not any(u.step(1.0, True) for _ in range(10)) and u.installed == 0
  finally:
    M.enabled = saved


def test_flag_path_is_under_data():
  import inspect
  assert M.ENABLE_FLAG == "/data/tn_autoupdate_enabled"
  assert "os.path.isfile(ENABLE_FLAG)" in inspect.getsource(M)


def test_main_off_in_d_fetches_at_once_and_installs_within_30_s():
  class Fetching(Probe):
    def __init__(self, params):
      super().__init__(params, pending=False)
      self.fetches = 0
    def fetch(self):
      self.fetches += 1
      self._pending = True  # the remote has a new commit
  u = Fetching(FakeParams())
  u.next_fetch = float("inf")  # the 5 min timer is far away
  for _ in range(5):
    u.step(1.0, False)
  assert u.fetches == 0
  t = 0
  while not u.installed and t < 30:
    u.step(1.0, True); t += 1
  assert u.fetches == 1 and u.installed == 1 and t <= M.HOLD_TIME + 1
