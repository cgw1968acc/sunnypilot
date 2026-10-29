# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Keep the set speed box (set speed, current speed, PCM ceiling) on screen during the lane change alerts (tncr, owner
2026-10-25: "the numbers must not disappear while changing lanes").

The mici HUD fades the box out under every alert because the alert banner covers the top of the screen. For the
lane change family the banner is a small one (a turn signal or blind spot icon and a short line), so the box stays and
the banner moves its text and left icon to the right of the box. Every other alert keeps the stock behaviour (the
box gives way so a warning is never partly covered). In the narrower room the banner font shrinks until its longest
word fits on one line (the label would otherwise break inside a word: "stee / r rig...").
"""
import math

from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

LANE_CHANGE_EVENTS = frozenset({
  'preLaneChangeLeft', 'preLaneChangeRight', 'laneChange', 'laneChangeBlocked', 'laneChangeRoadEdge',
})

_room = 0.0  # px the box takes from the left edge in the last frame it was drawn (0 when hidden)


def event_name(alert_type: str) -> str:
  # alert_type format is "EventName/eventType" (e.g., "preLaneChangeLeft/warning")
  return alert_type.split('/', maxsplit=1)[0] if alert_type else ''


def keeps_hud(alert) -> bool:
  return alert is not None and event_name(alert.alert_type) in LANE_CHANGE_EVENTS


def set_room(px: float) -> None:
  global _room
  _room = max(px, 0.0)


def make_room(event: str, icon_side, text_x: float, text_width: float, icon_margin_x: int,
              rect_x: float) -> tuple[float, float, int]:
  """alert text x/width and left icon margin with the box's room taken out (unchanged for any other alert)"""
  if event not in LANE_CHANGE_EVENTS or _room <= 0.0:
    return text_x, text_width, icon_margin_x
  room = math.ceil(_room)
  if icon_side == 'left':
    # icon sits right of the box, the right-aligned text keeps its place after the icon
    return text_x + room, text_width - room, icon_margin_x + room
  new_x = max(text_x, rect_x + room)
  return new_x, text_width - (new_x - text_x), icon_margin_x


FIT_MIN_FONT = 30  # px
FIT_STEP = 2  # px


def fit_font(alert_type: str, text: str, font_size: int, width: float, weight: FontWeight) -> int:
  """largest font size up to font_size whose longest word fits the width (unchanged for any other alert)"""
  if event_name(alert_type) not in LANE_CHANGE_EVENTS or _room <= 0.0 or not text:
    return font_size
  font = gui_app.font(weight)
  words = text.split()
  while font_size > FIT_MIN_FONT and max(measure_text_cached(font, w, font_size).x for w in words) > width - 4:
    font_size -= FIT_STEP
  return font_size
