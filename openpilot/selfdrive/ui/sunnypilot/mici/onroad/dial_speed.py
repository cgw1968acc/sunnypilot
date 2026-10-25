"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Current speed under the set speed, on the instrument cluster's scale (tncr, owner 2026-10-25: "the true speed in
place of MAX, in sync with the speedometer").

With the Toyota software set speed the set speed on the HUD is already the speedometer number
(cruise_ext.toyota_dial_from_canonical: dial = 1.0677 * vEgo + 2.53 km/h, Corolla Cross route 47 2026-09-25, road check
80 -> 80, 86 -> 86, 107 -> 107). The current speed uses the same line so both numbers match the dial. The line was
fitted at 26-128 km/h; its +2.53 offset cannot hold at a standstill (the dial reads 0), so the offset fades in
linearly up to OFFSET_FULL_KPH - unverified below 30 km/h, to be checked against the dial on the road.
Without the software set speed (no dial calibration) the stock HUD speed is shown.

The dial changes its digit slowly; the HUD updates every frame, so a hysteresis keeps the number from flickering
between two integers: it only moves once the value is HYST away from the number shown.
"""
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.cruise_ext import TOYOTA_DIAL_SLOPE, TOYOTA_DIAL_OFFSET_KPH

OFFSET_FULL_KPH = 30.0  # km/h (vEgo) from which the full dial offset applies
HYST = 0.6  # display units


def dial_kph(v_ego_kph: float) -> float:
  v = max(v_ego_kph, 0.0)
  return TOYOTA_DIAL_SLOPE * v + TOYOTA_DIAL_OFFSET_KPH * min(v / OFFSET_FULL_KPH, 1.0)


def software_set_speed(CP, CP_SP) -> bool:
  """openpilot owns the set speed and the PCM number is only a ceiling (same test as the HUD's PCM line)"""
  return CP is not None and CP_SP is not None and CP.pcmCruise and not CP_SP.pcmCruiseSpeed


class DialSpeed:
  def __init__(self):
    self.shown = 0

  def update(self, v_ego: float, stock_speed: float, dial: bool, is_metric: bool) -> int:
    """v_ego in m/s, stock_speed already in display units; returns the number to show"""
    if dial:
      value = dial_kph(v_ego * CV.MS_TO_KPH)
      if not is_metric:
        value *= CV.KPH_TO_MPH
    else:
      value = max(stock_speed, 0.0)
    if abs(value - self.shown) >= HYST or value < 0.5:
      self.shown = int(round(value))
    return self.shown
