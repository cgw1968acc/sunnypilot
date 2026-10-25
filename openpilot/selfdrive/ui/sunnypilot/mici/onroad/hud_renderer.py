"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer, FONT_SIZES, PCM_GAP
from openpilot.selfdrive.ui.sunnypilot.mici.onroad import lane_change_hud
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.dial_speed import DialSpeed, software_set_speed
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached

SPEED_LABEL_SIZE = 72  # px, current speed in place of MAX: twice the stock MAX size (owner 2026-10-25: 36 too small to read)
BOX_MARGIN = 10  # px kept free right of the set speed box for a lane change banner (lane_change_hud.py)


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self._dial_speed = DialSpeed()
    self.dial_speed: int = 0

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()
    sm = ui_state.sm
    if sm.recv_frame["carState"] >= ui_state.started_frame:
      dial = software_set_speed(ui_state.CP, getattr(ui_state, "CP_SP", None))
      self.dial_speed = self._dial_speed.update(sm['carState'].vEgo, self.speed, dial, ui_state.is_metric)

  def speed_label_sp(self) -> str:
    """current speed on the speedometer's scale, drawn where the stock HUD writes MAX (dial_speed.py)"""
    return str(self.dial_speed)

  def speed_label_size_sp(self) -> int:
    return SPEED_LABEL_SIZE

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    lane_change_hud.set_room(self._box_width() if self.drawing_top_icons() else 0.0)

  def _box_width(self) -> float:
    """width of the set speed box as drawn by HudRenderer._draw_set_speed (same x offsets and fonts)"""
    set_speed = self.set_speed if ui_state.is_metric else self.set_speed * CV.KPH_TO_MPH
    w = 17 + measure_text_cached(self._font_display, str(round(set_speed)), FONT_SIZES.set_speed).x
    label = 25 + measure_text_cached(self._font_semi_bold, self.speed_label_sp(), SPEED_LABEL_SIZE).x
    if self.pcm_ceiling > 0:  # drawn right of the label (HudRenderer._draw_set_speed)
      ceiling = self.pcm_ceiling if ui_state.is_metric else self.pcm_ceiling * CV.KPH_TO_MPH
      label += PCM_GAP + max(measure_text_cached(self._font_medium, t, FONT_SIZES.pcm_ceiling).x
                             for t in ("PCM", str(round(ceiling))))
    w = max(w, label)
    return w + BOX_MARGIN

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
