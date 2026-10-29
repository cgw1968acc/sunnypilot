# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import calendar
import json

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.night_gap import NightGap, sun_elevation, NIGHT_EXTRA_M


def unix(y, mo, d, h, mi):  # UTC
  return calendar.timegm((y, mo, d, h, mi, 0))


class P:
  def __init__(self, d): self.d = d
  def get(self, k): return self.d.get(k)


def test_sun_elevation_taipei():
  # Taipei 2026-10-12: sunrise ~05:50, sunset ~17:35 local (UTC+8)
  assert sun_elevation(unix(2026, 10, 12, 4, 0), 25.03, 121.56) > 30    # noon
  assert sun_elevation(unix(2026, 10, 11, 16, 30), 25.03, 121.56) < -30  # 00:30
  assert sun_elevation(unix(2026, 10, 12, 10, 0), 25.03, 121.56) < 0     # 18:00, after sunset
  assert sun_elevation(unix(2026, 10, 12, 9, 0), 25.03, 121.56) > 0      # 17:00, before sunset
  assert sun_elevation(unix(2026, 10, 12, 22, 30), 25.03, 121.56) > 0    # 06:30 next morning


def test_extra_only_at_night_and_at_highway_speed():
  v = np.array([60, 75, 80, 110]) / 3.6
  ng = NightGap(P({}), clock=lambda: unix(2026, 10, 11, 16, 30), monotonic=lambda: 0.0)
  assert ng.update()
  assert np.allclose(ng.extra(v), [0.0, NIGHT_EXTRA_M / 2, NIGHT_EXTRA_M, NIGHT_EXTRA_M])
  day = NightGap(P({}), clock=lambda: unix(2026, 10, 12, 4, 0), monotonic=lambda: 0.0)
  assert not day.update() and np.allclose(day.extra(v), 0.0)


def test_uses_the_stored_position_and_ignores_an_unset_clock():
  pos = P({"LastGPSPosition": json.dumps({"latitude": 25.03, "longitude": 121.56})})
  assert NightGap(pos, clock=lambda: unix(2026, 10, 11, 16, 30), monotonic=lambda: 0.0).update()
  assert not NightGap(pos, clock=lambda: 1.0e9, monotonic=lambda: 0.0).update()
  bad = P({"LastGPSPosition": "{}"})
  assert NightGap(bad, clock=lambda: unix(2026, 10, 11, 16, 30), monotonic=lambda: 0.0).update()  # default position
