"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Cruise soft start: gentle build-up of the cruise acceleration target when the car starts accelerating from a
steady cruise (no lead). The stock planner lets the target rise at J_CRUISE (~1.0-1.4 m/s^3), so a large set speed
step reaches the full target within 0.5-0.8 s and the Toyota PCM then overshoots it in the first second
(option1b_findings14 section 16). Here the upward jerk is limited to SOFT_START_J_UP[0] for the first second after
leaving idle and blends back to J_CRUISE over the second one. Downward jerk and the recovery from deceleration up to
idle are not touched, so releasing acceleration and coming off the brakes stay as responsive as before.
"""
import numpy as np

ACCEL_IDLE_THRESHOLD = 0.05  # m/s^2, below this the cruise target counts as "not accelerating"
SOFT_START_T_BP = [0.0, 1.0, 1.5, 2.5]  # s since the cruise target left idle
SOFT_START_J_UP = [0.15, 0.30, 0.60, 100.0]  # m/s^3 upward jerk limit; the last value hands control back to J_CRUISE


def soft_start_up_jerk(t_accelerating: float | None, j_cruise: float) -> float:
  """Upward jerk limit to use this cycle. None means the soft start is not armed (plain J_CRUISE)."""
  if t_accelerating is None:
    return j_cruise
  return float(min(j_cruise, np.interp(t_accelerating, SOFT_START_T_BP, SOFT_START_J_UP)))


def apply_cruise_jerk_limits(target_accel: float, a_prev: float, j_cruise: float, j_up: float, dt: float) -> float:
  """Rate limit the cruise target. Below idle the plain J_CRUISE applies in both directions so a deceleration is
  released quickly; once the target is above idle, rising is limited to j_up while falling keeps J_CRUISE."""
  up = j_cruise if a_prev < ACCEL_IDLE_THRESHOLD else j_up
  return float(np.clip(target_accel, a_prev - j_cruise * dt, a_prev + up * dt))


class CruiseSoftStart:
  def __init__(self, dt: float):
    self.dt = dt
    self.t_accelerating: float | None = None

  def reset(self) -> None:
    self.t_accelerating = None

  def update(self, a_cruise_prev: float) -> float | None:
    """Track how long the cruise target has been above idle. Returns the time to feed soft_start_up_jerk."""
    if a_cruise_prev < ACCEL_IDLE_THRESHOLD:
      self.t_accelerating = 0.0
    elif self.t_accelerating is not None:
      self.t_accelerating += self.dt
    return self.t_accelerating
