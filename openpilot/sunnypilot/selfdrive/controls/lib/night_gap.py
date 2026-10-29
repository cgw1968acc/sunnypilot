# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import math
import time

import numpy as np

# Night following gap (owner 2026-10-12): "at night at highway speed, now that the radar sticks to the lead, add a
# little following distance to be safe" -> +5 m (owner's choice of +5 / +10 m) from 80 km/h, faded in from 70 km/h.
# At 100 km/h in Relaxed the steady gap goes from ~56 m to ~61 m (+0.18 s). T_FOLLOW stays untouched (owner rule);
# the metres are added through the same per-node distance trim as the rest of long_mpc_ext.
# Night = the sun below the horizon (sunset/sunrise, -0.833 deg) at the car's position, from the device clock (GPS/NTP
# synced). Position from the in-memory LastGPSPosition when mapd keeps it, else a fixed point in Taiwan (sunset moves
# only a few minutes across the island).
NIGHT_EXTRA_M = 5.0
NIGHT_V_BP = [70.0 / 3.6, 80.0 / 3.6]  # m/s
NIGHT_V_W = [0.0, 1.0]
SUN_NIGHT_ELEV = -0.833  # deg
DEFAULT_LAT, DEFAULT_LON = 24.0, 121.0
CHECK_PERIOD = 60.0  # s
MIN_VALID_UNIX = 1.7e9  # a clock before 2023-11 is not set yet: no night extra


def sun_elevation(unix_t: float, lat: float, lon: float) -> float:
  """solar elevation in degrees (NOAA approximation, ~0.5 deg)"""
  tm = time.gmtime(unix_t)
  n = tm.tm_yday
  hours = tm.tm_hour + tm.tm_min / 60.0 + tm.tm_sec / 3600.0
  g = 2.0 * math.pi / 365.0 * (n - 1 + (hours - 12.0) / 24.0)
  eqtime = 229.18 * (0.000075 + 0.001868 * math.cos(g) - 0.032077 * math.sin(g) - 0.014615 * math.cos(2 * g)
                     - 0.040849 * math.sin(2 * g))
  decl = (0.006918 - 0.399912 * math.cos(g) + 0.070257 * math.sin(g) - 0.006758 * math.cos(2 * g)
          + 0.000907 * math.sin(2 * g) - 0.002697 * math.cos(3 * g) + 0.00148 * math.sin(3 * g))
  true_solar_min = hours * 60.0 + eqtime + 4.0 * lon
  ha = math.radians(true_solar_min / 4.0 - 180.0)
  la = math.radians(lat)
  cos_zen = math.sin(la) * math.sin(decl) + math.cos(la) * math.cos(decl) * math.cos(ha)
  return 90.0 - math.degrees(math.acos(max(-1.0, min(1.0, cos_zen))))


class NightGap:
  def __init__(self, mem_params=None, clock=time.time, monotonic=time.monotonic):
    self.mem_params = mem_params
    self.clock = clock
    self.monotonic = monotonic
    self.night = False
    self._next = 0.0

  def _position(self) -> tuple[float, float]:
    try:
      raw = self.mem_params.get("LastGPSPosition") if self.mem_params is not None else None
      if raw:
        d = json.loads(raw)
        lat, lon = float(d["latitude"]), float(d["longitude"])
        if abs(lat) > 1e-3 or abs(lon) > 1e-3:
          return lat, lon
    except Exception:
      pass
    return DEFAULT_LAT, DEFAULT_LON

  def update(self) -> bool:
    now = self.monotonic()
    if now >= self._next:
      self._next = now + CHECK_PERIOD
      t = self.clock()
      if t < MIN_VALID_UNIX:
        self.night = False
      else:
        lat, lon = self._position()
        self.night = sun_elevation(t, lat, lon) < SUN_NIGHT_ELEV
    return self.night

  def extra(self, v_plan) -> np.ndarray:
    """metres to ADD to the desired distance per node (0 by day)"""
    w = np.interp(np.maximum(v_plan, 0.0), NIGHT_V_BP, NIGHT_V_W)
    return NIGHT_EXTRA_M * w if self.night else np.zeros_like(w)
