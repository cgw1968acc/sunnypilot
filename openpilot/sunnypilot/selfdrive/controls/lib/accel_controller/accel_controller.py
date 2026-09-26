"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.sunnypilot import get_sanitize_int_param

AccelProfile = custom.LongitudinalPlanSP.AccelController.Profile

# Breakpoints at 60/70/80 km/h were added for the Corolla Cross Hybrid (2026-09-25). Normal/sport hold their
# previous straight-line values at the new points; eco below is its previous line too (0.40 @ 43 km/h -> 0.15 @ 86),
# used whenever the engine is already running (or the car does not report engine state).
MAX_ACCEL_BREAKPOINTS = [0., 3., 12., 16.67, 19.44, 22.22, 24., 36.]  # m/s (16.67/19.44/22.22 = 60/70/80 km/h)
MAX_ACCEL_PROFILES = {
  AccelProfile.eco:    [1.70, 1.38, 0.40, 0.30, 0.245, 0.19, 0.15, 0.10],
  AccelProfile.normal: [1.80, 1.55, 0.55, 0.48, 0.43, 0.39, 0.36, 0.25],
  AccelProfile.sport:  [2.00, 2.00, 1.20, 1.00, 0.89, 0.78, 0.70, 0.50],
}
# Eco while the hybrid's engine is OFF: keep wheel power m*a*v + road load (m=1500 kg, Crr 0.010, CdA 0.82) under the
# ~11-14 kW at which the Corolla Cross starts its engine (30 rlogs, 122 accel-driven starts, SOC 35-60%; the threshold
# does not move with SOC, rises mildly with speed: p10 14 kW at 80-89 km/h). 60 km/h 0.25, 70 km/h 0.16, 80 km/h 0.12.
# Once the engine runs there is nothing to save, so the normal eco line above takes over until it stops again.
ECO_ENGINE_OFF_MAX_ACCEL = [1.70, 1.38, 0.40, 0.25, 0.16, 0.12, 0.08, 0.06]
# Eco with NO lead: there is nothing to keep up with, so use a little less throttle again (driver request 2026-09-25,
# "再節能一些"). Faded in between 3 and 8 m/s so the launch from a stop is untouched.
ECO_NO_LEAD_FACTOR_BP = [3.0, 8.0]   # m/s
ECO_NO_LEAD_FACTOR_V = [1.0, 0.85]
# Cruise deceleration to a lowered set speed (no lead). Normal/sport: one CONSTANT decel latched when the gap opens
# (gap / response time, never gentler than CRUISE_DECEL_ACCEL) and held, so the car slows on a straight line instead of
# the proportional gap/time law, which braked hardest at the start and dragged a long tail; more presses = firmer.
# Eco (driver request 2026-09-25): never hurry to the new set speed however far it is or however many presses - lift
# off and let the car slow. Above 70 km/h the target is the car's own coast decel (road load of a 1500 kg Corolla
# Cross, Crr 0.010, CdA 0.82: -0.22 at 70, -0.30 at 90, -0.40 at 120 km/h), below 60 km/h -0.40 (regen; -0.30 until
# the 2026-09-26 trial), blended between 70 and 60 as the speed falls. A function of speed only: the line stays straight,
# the transition from coasting is continuous. Whether -0.40 is too firm is for the road test.
CRUISE_DECEL_RESPONSE_TIME = {  # seconds to close the gap (normal/sport)
  AccelProfile.normal: 3.5,
  AccelProfile.sport: 3.0,
}
CRUISE_DECEL_ACCEL = {  # m/s^2; gentlest constant decel (normal/sport)
  AccelProfile.normal: -0.50,
  AccelProfile.sport: -0.65,
}
ECO_CRUISE_DECEL_BP = [0., 16.67, 19.44, 25.0, 33.3]   # m/s (0, 60, 70, 90, 120 km/h)
ECO_CRUISE_DECEL_V = [-0.40, -0.40, -0.22, -0.30, -0.40]  # m/s^2 (<=60 km/h -0.30 -> -0.40, driver trial 2026-09-26)
CRUISE_DECEL_TAPER_TIME = 1.0  # s; inside |decel| * this of the target the decel eases off proportionally (no overshoot)


class AccelController:
  def __init__(self):
    self.params = Params()
    self._cruise_decel: float | None = None
    self._cruise_decel_target: float | None = None
    self.update()

  def update(self) -> None:
    self._profile = get_sanitize_int_param("AccelPersonality", AccelProfile.eco, AccelProfile.sport, self.params)
    self._enabled = self.params.get_bool("AccelPersonalityEnabled")

  @property
  def profile(self) -> int:
    return self._profile

  def is_enabled(self) -> bool:
    return self._enabled

  def get_max_accel(self, v_ego: float, engine_off: bool = False, has_lead: bool = True) -> float:
    profile = ECO_ENGINE_OFF_MAX_ACCEL if (engine_off and self._profile == AccelProfile.eco) else MAX_ACCEL_PROFILES[self._profile]
    max_accel = float(np.interp(max(0.0, v_ego), MAX_ACCEL_BREAKPOINTS, profile))
    if self._profile == AccelProfile.eco and not has_lead:
      max_accel *= float(np.interp(v_ego, ECO_NO_LEAD_FACTOR_BP, ECO_NO_LEAD_FACTOR_V))
    return max_accel

  def get_cruise_target(self, v_ego: float, v_target: float) -> float:
    if not np.isfinite(v_target) or v_target <= 0.0 or v_target >= v_ego:
      self._cruise_decel = None
      self._cruise_decel_target = None
      return v_target

    target_delta = v_target - v_ego
    if self._profile == AccelProfile.eco:
      # speed-scheduled, independent of the gap and of how many presses opened it
      decel = float(np.interp(v_ego, ECO_CRUISE_DECEL_BP, ECO_CRUISE_DECEL_V))
    else:
      if self._cruise_decel is None or self._cruise_decel_target != v_target:
        # a new (or changed) gap: latch one constant decel for this episode
        self._cruise_decel = min(CRUISE_DECEL_ACCEL[self._profile], target_delta / CRUISE_DECEL_RESPONSE_TIME[self._profile])
        self._cruise_decel_target = v_target
      decel = self._cruise_decel

    # taper only once the target is close (both terms are negative; max = the gentler one) so the speed lands cleanly
    decel = max(decel, target_delta / CRUISE_DECEL_TAPER_TIME)
    return float(v_ego + decel)
