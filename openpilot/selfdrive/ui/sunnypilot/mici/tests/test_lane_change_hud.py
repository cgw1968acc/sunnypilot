# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from openpilot.selfdrive.ui.sunnypilot.mici.onroad import lane_change_hud as lch


def test_keeps_hud_only_for_lane_change_alerts():
  for ev in ('preLaneChangeLeft', 'preLaneChangeRight', 'laneChange', 'laneChangeBlocked', 'laneChangeRoadEdge'):
    assert lch.keeps_hud(SimpleNamespace(alert_type=f'{ev}/warning'))
  for ev in ('steerSaturated/warning', 'promptDriverDistracted/permanent', ''):
    assert not lch.keeps_hud(SimpleNamespace(alert_type=ev))
  assert not lch.keeps_hud(None)


def test_make_room():
  lch.set_room(200.4)
  # no icon / right icon: text starts right of the box
  assert lch.make_room('laneChange', None, 18, 458, 20, 0) == (201, 275, 20)
  # left icon: icon and text both move by the room
  assert lch.make_room('preLaneChangeLeft', 'left', 104, 354, 2, 0) == (305, 153, 203)
  # other alerts untouched
  assert lch.make_room('steerSaturated', None, 18, 458, 20, 0) == (18, 458, 20)
  # box hidden: untouched
  lch.set_room(0.0)
  assert lch.make_room('laneChange', None, 18, 458, 20, 0) == (18, 458, 20)
