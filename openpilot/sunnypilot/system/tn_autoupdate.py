#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Drive-gear auto update (owner 2026-10-11): "update automatically when the car is in D with MAIN ACC off and the device
has a connection - I am an expert user, I accept the risk", restarting with `sudo systemctl restart comma` (not a full
reboot) so openpilot is back quickly.

The stock updater only runs offroad and installs at the next boot, and the C3X rarely reboots, so the owner has been
updating by hand over SSH (fetch + reset --hard + restart). This process does the same thing on its own:
  - every FETCH_PERIOD it fetches the target branch (UpdaterTargetBranch, else the checked-out branch) from origin;
  - when the fetched commit differs from HEAD, the working tree is clean, and the car has been in D with MAIN ACC off and
    openpilot not engaged for HOLD_TIME, it resets the checkout to that commit, drops any update the stock updater has
    staged (the launch script would otherwise swap it in), and restarts the comma service.
While openpilot restarts its steering / ACC messages stop and the car may show a system warning until the next
ignition cycle. DisableUpdates=1 turns it off.
"""
import subprocess
import shutil
import time

from opendbc.car.structs import car
import openpilot.cereal.messaging as messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

FETCH_PERIOD = 300.  # s
FIRST_FETCH_DELAY = 30.  # s after start
FETCH_TIMEOUT = 120.  # s
HOLD_TIME = 3.0  # s the drive conditions must hold before the restart
STAGING_FINALIZED = "/data/safe_staging/finalized"
RESTART_CMD = ["sudo", "systemctl", "restart", "comma"]


def drive_gear_main_off(gear, cruise_available: bool, engaged: bool) -> bool:
  return gear == car.CarState.GearShifter.drive and not cruise_available and not engaged


def git(*args, timeout=30.) -> str:
  return subprocess.check_output(["git", "-C", BASEDIR, *args], text=True, timeout=timeout,
                                 stderr=subprocess.STDOUT).strip()


class DriveGearUpdater:
  def __init__(self, params: Params):
    self.params = params
    self.remote_sha: str | None = None
    self.next_fetch = time.monotonic() + FIRST_FETCH_DELAY
    self.hold_t = 0.0

  def branch(self) -> str:
    return self.params.get("UpdaterTargetBranch") or git("rev-parse", "--abbrev-ref", "HEAD")

  def fetch(self) -> None:
    try:
      b = self.branch()
      git("fetch", "--quiet", "origin", f"+{b}:refs/remotes/origin/{b}", timeout=FETCH_TIMEOUT)
      self.remote_sha = git("rev-parse", f"refs/remotes/origin/{b}")
    except Exception as e:  # no connection, timeout, branch gone: try again next period
      cloudlog.warning(f"tn_autoupdate: fetch failed: {e}")

  def pending(self) -> bool:
    if not self.remote_sha:
      return False
    try:
      return self.remote_sha != git("rev-parse", "HEAD") and git("status", "--porcelain", "--untracked-files=no") == ""
    except Exception:
      return False

  def install(self) -> None:
    sha = self.remote_sha
    cloudlog.event("tn_autoupdate: installing", sha=sha, error=True)
    b = self.branch()
    git("checkout", "-q", "-B", b, sha, timeout=120.)
    shutil.rmtree(STAGING_FINALIZED, ignore_errors=True)
    self.params.put_bool("UpdateAvailable", False)
    subprocess.run(["sync"], timeout=30)
    # systemd stops and restarts every openpilot process, this one included
    subprocess.Popen(RESTART_CMD, start_new_session=True)

  def step(self, dt: float, condition: bool) -> bool:
    """returns True when it started an install"""
    now = time.monotonic()
    if self.params.get_bool("DisableUpdates"):
      self.hold_t = 0.0
      return False
    if now >= self.next_fetch:
      self.next_fetch = now + FETCH_PERIOD
      self.fetch()
    self.hold_t = self.hold_t + dt if condition else 0.0
    if self.hold_t >= HOLD_TIME and self.pending():
      self.install()
      return True
    return False


def main():
  params = Params()
  sm = messaging.SubMaster(['carState', 'selfdriveState'])
  updater = DriveGearUpdater(params)
  while True:
    sm.update(0)
    alive = sm.alive['carState'] and sm.alive['selfdriveState']
    cs, ss = sm['carState'], sm['selfdriveState']
    condition = alive and drive_gear_main_off(cs.gearShifter, cs.cruiseState.available, ss.enabled)
    if updater.step(1.0, condition):
      time.sleep(60.)  # the restart takes this process down
    time.sleep(1.0)


if __name__ == "__main__":
  main()
