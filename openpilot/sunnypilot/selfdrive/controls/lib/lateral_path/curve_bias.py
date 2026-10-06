"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.constants import CV

# Corner-cutting fix (Altis rlog 2026-09-20: the e2e path sits ~0.6 m inside on curves, both directions). Relax the
# commanded curvature by a fraction so the car runs a little wider (outward) in proportion to how tight the curve is.
# Straights and small lane corrections below the deadzone are untouched. Schedule ported from tncr18 (Corolla Cross
# road tests 2026-09-23..25): the outward bias only helps at higher speed, where the path cuts the inside. On tight
# LOW-speed curves the car tends to run WIDE instead, so relaxing the curvature there makes it worse.
CURVE_OUTWARD_DEADZONE = 0.0005  # 1/m (~radius 2000 m): below this is a straight / small correction, untouched.
# Speed schedule (m/s -> fraction): 0 up to 40 km/h, 0.09 at 70, 0.155 at 90 km/h, 0.19 at 120 km/h.
CURVE_OUTWARD_V_BP = [v * CV.KPH_TO_MS for v in (30., 40., 50., 60., 70., 90., 100., 120.)]   # m/s
# Negative = INWARD bias (command more curvature than the model): Altis driver 2026-09-27, 30-40 km/h curves want more
# steering; -0.05 at 30 km/h, then the outward schedule from 40 km/h. The hand-over from inward to outward is by
# interpolation between 30 and 40 km/h (zero at ~37 km/h).
CURVE_OUTWARD_FRAC_V = [0.0, 0.02, 0.06, 0.085, 0.09, 0.155, 0.155, 0.155]  # 2026-09-30: 60 km/h point 0.085 (+0.01 over the
# interpolated 0.075, driver request). 2026-09-29 driver request: 40 km/h 0 -> 0.02,
# 50 km/h 0.05 -> 0.06, flat at the 90 km/h value 0.155 from 90 upward (100/120 were 0.167).
# 2026-10-06 driver request: 30 km/h -0.05 -> 0 (no inward bias at or below 30 km/h; ramps to 0.02 at 40).


def apply_curve_outward_bias(desired_curvature: float, v_ego: float) -> float:
  # In a fast curve, command a slightly smaller curvature so the car runs wider and stops cutting the inside. Faded
  # out at low speed (tight curves there need the full turn-in or the car runs wide). Straights and small lane
  # corrections below the deadzone are untouched.
  frac = float(np.interp(v_ego, CURVE_OUTWARD_V_BP, CURVE_OUTWARD_FRAC_V))
  if frac == 0.0 or abs(desired_curvature) <= CURVE_OUTWARD_DEADZONE:
    return desired_curvature
  return desired_curvature * (1.0 - frac)   # frac < 0 scales the curvature UP (inward)
