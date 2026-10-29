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
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.dial_speed import DialSpeed, dial_kph


def _shown(v_ego_kph, dial=True, is_metric=True, stock=0.0, ds=None):
  ds = ds or DialSpeed()
  return ds.update(v_ego_kph * CV.KPH_TO_MS, stock, dial, is_metric)


def test_matches_the_road_checked_dial():
  # Corolla Cross 2026-09-25: vEgo -> speedometer
  for v, dial in [(44.3, 50), (72.4, 80), (78.4, 86), (91.1, 100), (97.9, 107)]:
    assert _shown(v) == dial


def test_standstill_reads_zero_and_offset_fades_in():
  assert _shown(0.0) == 0
  assert dial_kph(0.0) == 0.0
  assert dial_kph(15.0) < 1.0677 * 15.0 + 2.53
  assert abs(dial_kph(30.0) - (1.0677 * 30.0 + 2.53)) < 0.01
  prev = -1.0
  for i in range(1400):
    d = dial_kph(i / 10)
    assert d >= prev
    prev = d


def test_hysteresis_keeps_the_number_until_it_moves_0_6():
  def v_for(dial):  # vEgo kph giving this dial value (above 30 km/h)
    return (dial - 2.53) / 1.0677
  ds = DialSpeed()
  assert _shown(v_for(80.0), ds=ds) == 80
  assert _shown(v_for(80.55), ds=ds) == 80
  assert _shown(v_for(79.45), ds=ds) == 80
  assert _shown(v_for(80.65), ds=ds) == 81
  assert _shown(v_for(80.45), ds=ds) == 81  # 0.55 below 81: stays
  assert _shown(v_for(80.35), ds=ds) == 80


def test_stock_speed_without_software_set_speed():
  assert _shown(90.0, dial=False, stock=91.4) == 91


def test_mph():
  assert _shown(91.1, is_metric=False) == round(100 * CV.KPH_TO_MPH)
