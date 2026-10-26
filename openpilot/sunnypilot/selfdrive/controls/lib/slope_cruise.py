"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.constants import CV

# Slope cruise (tncr, Corolla Cross; owner 2026-10-17 night): "on the highway, on a slight climb or descent, in relaxed
# and eco, control the speed by itself: downhill allow the set speed +10 km/h, never above 120; uphill allow -10 km/h,
# never below 70; speed up and slow down smoothly". So with the distance button at relaxed and the accel personality at
# eco, above V_MIN, the planner's cruise target becomes set speed + an offset from the road grade: 0 inside
# +-GRADE_DEAD, the full +-OFFSET_KPH at +-GRADE_FULL (descent = +, climb = -). The offset moves at most OFFSET_RATE (a
# 10 km/h change over ~30 s = ~0.09 m/s^2 of speed change), so the car drifts up on a descent mostly on gravity and
# drifts down on a climb instead of pulling harder; leaving the slope it drifts back the same way. The set speed shown
# on the dash is not changed; a lead still governs through the MPC as always.
# Grade = the car's pitch (carControl.orientationNED, nose up +) minus its flat-road offset. Against the GPS altitude
# (route 00000084 / 00000086, 10 s windows > 60 km/h) the pitch is 0.92 / 0.84 x the grade (corr 0.96 / 0.82) with an
# offset of +0.37 / +0.73 deg that differs per drive, so the offset is learnt: a slow low-pass of the pitch while above
# V_MIN (OFFSET_TAU, starting at OFFSET_INIT_DEG) - a highway averages out flat over minutes, a slope does not move it.
ENABLED = True
# tncr, owner 2026-10-27 (route b8 23:09): the descent allowance is REMOVED - on a mountain road at 63 km/h (dash 70)
# it let the car run to 73.6 km/h (dash ~80) going into a curve. Only the climb part is kept, and only on the highway:
# V_MIN 60 -> 80 km/h (60 let it act on mountain roads).
V_MIN = 80.0 / 3.6          # m/s: highway speeds only
OFFSET_DOWN_KPH = 0.0       # km/h on a descent (was +10)
OFFSET_KPH = 10.0           # km/h on a climb (-)
V_HI_KPH = 120.0            # downhill: never above (unless the set speed itself is higher)
V_LO_KPH = 70.0             # uphill: never below (unless the set speed itself is lower)
GRADE_DEAD = 0.6            # deg: a "flat" road, no change
GRADE_FULL = 1.5            # deg (~2.6 %): the full offset
GRADE_TAU = 2.0             # s: low-pass of the pitch
OFFSET_TAU = 300.0          # s: learning of the flat-road pitch
OFFSET_INIT_DEG = 0.5       # deg: start value (the two drives measured +0.37 / +0.73)
OFFSET_RATE = 0.33          # km/h per s
DT = 0.05                   # s, planner step


class SlopeCruise:
  def __init__(self, dt: float = DT):
    self.dt = dt
    self.flat = OFFSET_INIT_DEG
    self.pitch_f: float | None = None
    self.offset = 0.0  # km/h

  def grade_deg(self) -> float:
    return 0.0 if self.pitch_f is None else self.pitch_f - self.flat

  def update(self, v_cruise: float, v_ego: float, pitch_rad: float | None, active: bool) -> float:
    """v_cruise in m/s -> the cruise target in m/s"""
    pitch_ok = pitch_rad is not None and np.isfinite(pitch_rad)
    if pitch_ok and v_ego > V_MIN:
      p = float(np.degrees(pitch_rad))
      self.pitch_f = p if self.pitch_f is None else self.pitch_f + (p - self.pitch_f) * self.dt / GRADE_TAU
      self.flat += (p - self.flat) * self.dt / OFFSET_TAU
    target = 0.0
    if ENABLED and active and pitch_ok and v_ego > V_MIN and self.pitch_f is not None:
      g = self.grade_deg()
      target = float(np.interp(g, [-GRADE_FULL, -GRADE_DEAD, GRADE_DEAD, GRADE_FULL], [OFFSET_DOWN_KPH, 0.0, 0.0, -OFFSET_KPH]))
    step = OFFSET_RATE * self.dt
    self.offset = float(np.clip(target, self.offset - step, self.offset + step))
    if self.offset == 0.0 or not np.isfinite(v_cruise) or v_cruise <= 0.0:
      return v_cruise
    set_kph = v_cruise * CV.MS_TO_KPH
    out = set_kph + self.offset
    out = min(out, max(set_kph, V_HI_KPH)) if self.offset > 0.0 else max(out, min(set_kph, V_LO_KPH))
    return out * CV.KPH_TO_MS
