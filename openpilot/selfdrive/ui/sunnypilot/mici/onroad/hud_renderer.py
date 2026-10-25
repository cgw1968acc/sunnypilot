"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer, FONT_SIZES
from openpilot.selfdrive.ui.sunnypilot.mici.onroad import lane_change_hud
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.dial_speed import DialSpeed, software_set_speed
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached

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

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    lane_change_hud.set_room(self._box_width() if self.drawing_top_icons() else 0.0)

  def _box_width(self) -> float:
    """width of the set speed box as drawn by HudRenderer._draw_set_speed (same x offsets and fonts)"""
    set_speed = self.set_speed if ui_state.is_metric else self.set_speed * CV.KPH_TO_MPH
    w = 17 + measure_text_cached(self._font_display, str(round(set_speed)), FONT_SIZES.set_speed).x
    w = max(w, 25 + measure_text_cached(self._font_semi_bold, self.speed_label_sp(), FONT_SIZES.max_speed).x)
    if self.pcm_ceiling > 0:
      ceiling = self.pcm_ceiling if ui_state.is_metric else self.pcm_ceiling * CV.KPH_TO_MPH
      w = max(w, 25 + measure_text_cached(self._font_medium, f"PCM {round(ceiling)}", FONT_SIZES.pcm_ceiling).x)
    return w + BOX_MARGIN

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
