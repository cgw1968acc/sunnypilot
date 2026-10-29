# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
import numpy as np

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.mid_speed_brake_ease import mid_speed_brake_ease


def test_five_percent_at_30_to_50_kph():
  for kph in (30., 40., 50.):
    assert abs(mid_speed_brake_ease(-1.2, kph * CV.KPH_TO_MS) - (-1.14)) < 1e-9


def test_untouched_below_25_and_above_55_kph():
  for kph in (0., 5., 15., 25., 55., 80.):
    assert mid_speed_brake_ease(-1.0, kph * CV.KPH_TO_MS) == -1.0


def test_never_touches_acceleration():
  assert mid_speed_brake_ease(0.5, 40. * CV.KPH_TO_MS) == 0.5
  assert mid_speed_brake_ease(0.0, 40. * CV.KPH_TO_MS) == 0.0


def test_hard_request_not_eased_and_monotonic():
  v = 40. * CV.KPH_TO_MS
  assert mid_speed_brake_ease(-2.2, v) == -2.2
  assert mid_speed_brake_ease(-3.0, v) == -3.0
  out = [mid_speed_brake_ease(a, v) for a in np.linspace(-3.0, 0.0, 601)]
  assert all(b >= a for a, b in zip(out, out[1:], strict=False))  # firmer request -> never a lighter output
  assert all(o <= 0.0 for o in out)


def test_lighter_never_firmer():
  for kph in np.linspace(0., 80., 81):
    for a in np.linspace(-3.0, 0.0, 61):
      o = mid_speed_brake_ease(a, kph * CV.KPH_TO_MS)
      assert a <= o <= 0.0 and o - a <= 0.05 * abs(a) + 1e-9
