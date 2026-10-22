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
# tncr, owner 2026-10-15 night: "port tncr19's FRAC values and the other lateral corrections" - tncr was cut with the
# Altis (tnpb3) lateral values; the Corolla Cross keeps its own road-tested tune (tncr18 -> tncr19 693947f88b):
# -0.05 at 30 km/h (inward), 0.02 at 40, 0.06 at 50, 0.085 at 60, 0.09 at 70, flat 0.155 from 90 km/h.
# (Altis/tnpb3 values were -0.01 @30, 0 @35, 0.03 @40, 0.063 @50, 0.087 @60.)
CURVE_OUTWARD_V_BP = [v * CV.KPH_TO_MS for v in (30., 40., 50., 60., 70., 90., 100., 120.)]   # m/s
# Owner 2026-10-22 night: "raise FRAC at 50-60 km/h by 0.01" -> 50: 0.06 -> 0.07, 60: 0.085 -> 0.095 (70 stays 0.09).
CURVE_OUTWARD_FRAC_V = [-0.05, 0.02, 0.07, 0.095, 0.09, 0.155, 0.155, 0.155]


def apply_curve_outward_bias(desired_curvature: float, v_ego: float) -> float:
  # In a fast curve, command a slightly smaller curvature so the car runs wider and stops cutting the inside. Faded
  # out at low speed (tight curves there need the full turn-in or the car runs wide). Straights and small lane
  # corrections below the deadzone are untouched.
  frac = float(np.interp(v_ego, CURVE_OUTWARD_V_BP, CURVE_OUTWARD_FRAC_V))
  if frac == 0.0 or abs(desired_curvature) <= CURVE_OUTWARD_DEADZONE:
    return desired_curvature
  return desired_curvature * (1.0 - frac)   # frac < 0 scales the curvature UP (inward)
