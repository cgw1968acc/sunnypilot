"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Brake onset shaping for the Toyota PCM acceleration command. The stock controller lets the command fall at
ACCEL_WINDDOWN_LIMIT (4 m/s^3) from the first frame, so a new brake request goes from nothing to -1.2 m/s^2 in
0.3 s. On a light car with a sharp brake actuator (Corolla hybrid) that first 0.3 s feels like a stab, far from how
a driver eases onto the pedal. Here the downward jerk follows a schedule that restarts whenever the request starts
falling faster than the gentlest rate: almost nothing in the first 0.1 s, building through 0.2-0.3 s, and back to
the stock limit after ONSET_T_BP[-1]. Because the schedule keys off the *request*, a lead switch to a closer car
during a steady brake gets the same soft onset on top of the brake already applied. Hard braking requests and FCW
bypass the schedule entirely, and the upward (release) limit is not touched.
"""
import numpy as np

ONSET_T_BP = [0.0, 0.1, 0.2, 0.3, 0.45]  # s since the request began falling faster than ONSET_J_DOWN[0]
ONSET_J_DOWN = [0.4, 0.6, 1.5, 3.0, 4.0]  # m/s^3 downward jerk limit; last value equals the stock limit
HARD_BRAKE_ACCEL = -2.0  # m/s^2, requests below this are urgent and get the stock limit straight away


class BrakeOnsetShaper:
  def __init__(self, dt: float, stock_down_jerk: float):
    self.dt = dt
    self.stock_down_jerk = stock_down_jerk
    self.t_onset = 0.0

  def reset(self) -> None:
    self.t_onset = 0.0

  def down_step(self, accel_request: float, prev_accel: float, bypass: bool = False) -> float:
    """Negative per-frame step allowed for the command this cycle (feed as dw_step to rate_limit)."""
    if bypass:
      self.t_onset = 0.0
      return -self.stock_down_jerk * self.dt

    gentlest_step = -ONSET_J_DOWN[0] * self.dt
    onset = (accel_request - prev_accel) < gentlest_step - 1e-9
    if onset:
      j_down = float(np.interp(self.t_onset, ONSET_T_BP, ONSET_J_DOWN))
      self.t_onset = min(self.t_onset + self.dt, ONSET_T_BP[-1])
    else:
      # a gentle or rising request lets the schedule wind back at real time, so a short pause in a brake
      # does not fully re-arm it while a settled brake does
      j_down = self.stock_down_jerk
      self.t_onset = max(self.t_onset - self.dt, 0.0)
    return -min(j_down, self.stock_down_jerk) * self.dt

  def is_urgent(self, accel_request: float, fcw: bool) -> bool:
    return fcw or accel_request < HARD_BRAKE_ACCEL
