"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.end_force_limit import EndForceLimit, downhill_extra, END_ACCEL

KPH = 1 / 3.6


def _pitch(downhill_deg):
  """calibrated pitch (rad) for a grade this many degrees downhill of the flat reading (+1.0 deg)"""
  return math.radians(1.0 - downhill_deg)


def test_flat_uphill_and_dead_band_untouched():
  for d in (-5.0, -1.0, 0.0, 0.5):
    assert downhill_extra(_pitch(d), 10 * KPH, -1.0) == 0.0
  assert downhill_extra(None, 10 * KPH, -1.0) == 0.0
  assert downhill_extra(float('nan'), 10 * KPH, -1.0) == 0.0


def test_one_hundredth_per_degree_beyond_the_dead_band():
  assert abs(downhill_extra(_pitch(5.5), 10 * KPH, -1.0) - 0.05) < 1e-9
  assert abs(downhill_extra(_pitch(3.5), 20 * KPH, -0.5) - 0.03) < 1e-9
  assert downhill_extra(_pitch(30.0), 10 * KPH, -1.0) == 0.10  # cap


def test_only_below_30_kph_and_only_when_braking():
  assert downhill_extra(_pitch(5.5), 30 * KPH, -1.0) == 0.0
  assert abs(downhill_extra(_pitch(5.5), 27.5 * KPH, -1.0) - 0.025) < 1e-9
  assert downhill_extra(_pitch(5.5), 10 * KPH, 0.0) == 0.0
  assert downhill_extra(_pitch(5.5), 10 * KPH, 0.3) == 0.0
  assert abs(downhill_extra(_pitch(5.5), 10 * KPH, -0.05) - 0.025) < 1e-9


def test_continuous_in_speed_and_request():
  prev = None
  for v in np.linspace(35, 0, 351):
    e = downhill_extra(_pitch(8.0), v * KPH, -1.0)
    if prev is not None:
      assert abs(e - prev) < 0.002
    prev = e
  prev = None
  accels = np.linspace(0.2, -1.0, 1201)
  for a in accels:
    e = downhill_extra(_pitch(8.0), 10 * KPH, a)
    if prev is not None:
      assert abs(e - prev) <= 0.75 * abs(accels[1] - accels[0]) + 1e-9  # fades in over 0 .. -0.1 m/s^2, no step
    prev = e


def test_the_end_value_moves_by_the_same_amount():
  flat, hill = EndForceLimit(), EndForceLimit()
  pitch = _pitch(5.5)  # 0.05
  for v_kph in np.linspace(8, 0.05, 200):
    a_flat = flat.update(-1.2, v_kph * KPH, False, True, -1.0)
    a_hill = hill.update(-1.2, v_kph * KPH, False, True, -1.0, pitch=pitch)
    assert abs((a_flat - a_hill) - 0.05) < 1e-9
  assert abs(a_flat - END_ACCEL) < 0.03
  assert END_ACCEL == -0.34


def test_flat_road_identical_to_before():
  a, b = EndForceLimit(), EndForceLimit()
  for v_kph in np.linspace(30, 0, 300):
    assert a.update(-0.8, v_kph * KPH, False, True, -0.8) == b.update(-0.8, v_kph * KPH, False, True, -0.8, pitch=_pitch(0.0))


def test_inactive_untouched():
  assert EndForceLimit().update(-1.0, 10 * KPH, False, False, -1.0, pitch=_pitch(8.0)) == -1.0
