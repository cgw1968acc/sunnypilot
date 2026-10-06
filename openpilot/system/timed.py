#!/usr/bin/env python3
import datetime
import os
import subprocess
import time
from pathlib import Path
from typing import NoReturn

import openpilot.cereal.messaging as messaging
from openpilot.common.time_helpers import min_date, MAX_DATE, system_time_valid
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service


# fake-hwclock: this C3X's RTC does not hold time across a power loss (it reads 1970 at every boot, so systemd sets the
# clock to its own build date, 2026-07-28, until GPS or NTP fixes it ~1 min later). Remember the last valid time and
# restore it at boot, so the clock is only off by the parked duration instead of months: crash logs and log timestamps
# stay sane and TLS works right away. GPS/NTP still correct it afterwards.
LAST_TIME_FILE = Path("/data/last_known_time")
LAST_TIME_SAVE_PERIOD = 60  # s


def restore_last_known_time() -> None:
  try:
    saved = datetime.datetime.fromisoformat(LAST_TIME_FILE.read_text().strip())
  except (OSError, ValueError):
    return
  now = datetime.datetime.now(datetime.UTC).replace(tzinfo=None)
  if min_date() < saved < MAX_DATE and (not system_time_valid() or now < saved):
    cloudlog.warning(f"timed: system time {now} invalid or behind, restoring last known time {saved}")
    set_time(saved)


def save_last_known_time() -> None:
  if not system_time_valid():
    return
  try:
    tmp = LAST_TIME_FILE.with_suffix(".tmp")
    tmp.write_text(datetime.datetime.now(datetime.UTC).replace(tzinfo=None).isoformat())
    os.replace(tmp, LAST_TIME_FILE)
  except OSError:
    cloudlog.exception("timed: could not save last known time")


def set_time(new_time):
  diff = datetime.datetime.now(datetime.UTC).replace(tzinfo=None) - new_time
  if abs(diff) < datetime.timedelta(seconds=10):
    cloudlog.debug(f"Time diff too small: {diff}")
    return

  cloudlog.debug(f"Setting time to {new_time}")
  try:
    subprocess.run(f"TZ=UTC date -s '{new_time}'", shell=True, check=True)
  except subprocess.CalledProcessError:
    cloudlog.exception("timed.failed_setting_time")


def main() -> NoReturn:
  """
    timed has two responsibilities:
    - getting the current time from GPS
    - publishing the time in the logs

    AGNOS will also use NTP to update the time.
  """

  params = Params()
  gps_location_service = get_gps_location_service(params)

  pm = messaging.PubMaster(['clocks'])
  sm = messaging.SubMaster([gps_location_service])
  restore_last_known_time()
  last_save = 0.0
  while True:
    sm.update(1000)

    msg = messaging.new_message('clocks')
    msg.valid = system_time_valid()
    msg.clocks.wallTimeNanos = time.time_ns()
    pm.send('clocks', msg)

    if time.monotonic() - last_save > LAST_TIME_SAVE_PERIOD:
      save_last_known_time()
      last_save = time.monotonic()

    gps = sm[gps_location_service]
    gps_time = datetime.datetime.fromtimestamp(gps.unixTimestampMillis / 1000., datetime.UTC).replace(tzinfo=None)
    if not sm.updated[gps_location_service] or (time.monotonic() - sm.logMonoTime[gps_location_service] / 1e9) > 2.0:
      continue
    if not gps.hasFix:
      continue
    if gps_time < min_date() or gps_time > MAX_DATE:
      continue

    set_time(gps_time)
    time.sleep(10)

if __name__ == "__main__":
  main()
