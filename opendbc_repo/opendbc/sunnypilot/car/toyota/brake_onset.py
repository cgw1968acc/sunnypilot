"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Brake onset shaping for the Toyota PCM acceleration command. The stock controller lets the command fall at
ACCEL_WINDDOWN_LIMIT (4 m/s^3) from the first frame, so a new brake request goes from nothing to -1.2 m/s^2 in
0.3 s. On a light car with a sharp brake actuator (Corolla hybrid) that first 0.3 s feels like a stab, far from how
a driver eases onto the pedal. Here the downward jerk follows a schedule that restarts whenever the request starts
falling faster than the gentlest rate: almost nothing in the first 0.1 s, then one straight ramp of the jerk limit
up to the stock value at ONSET_T_BP[-1], so the soft start blends into the normal brake without a second bend
(Corolla Cross request 2026-09-26, route 53 seg 14: a lead brake at 95 km/h went 0 -> -0.3 m/s^2 in 0.3 s and
-0.9 in 1.1 s and felt heavy; the driver asked for almost nothing in the first 0.2 s and a slow build after it).
Because the schedule keys off the *request*, a lead switch to a closer car during a steady brake gets the same soft
onset on top of the brake already applied. Hard braking requests and FCW bypass the schedule entirely, and the upward (release) limit is not touched.
"""
import numpy as np

ONSET_T_BP = [0.0, 0.15, 0.6]  # (superseded by schedule_t(v) - see ONSET_T1_V / ONSET_T3_V)  # s since the request began falling faster than ONSET_J_DOWN[0] (at speed)
# At low speed the join is slower (Altis 2026-09-27 22:27, engage at 35 km/h behind a close lead: the ramp was at
# ~1.7 m/s^3 by 0.3 s and the PCM over-delivered to -1.1 with -7.9 m/s^3 jerk). Below 40 km/h the schedule's time
# points stretch to 0.3 / 1.2 s, above 60 km/h they are the stock 0.15 / 0.6 s, interpolated in between.
ONSET_V_BP = [11.1, 16.7]  # m/s (40, 60 km/h)
ONSET_T1_V = [0.1, 0.1]
ONSET_T3_V = [0.3, 0.3]
ONSET_J_DOWN = [1.0, 1.0, 4.0]  # m/s^3 downward jerk limit: almost nothing for the first instant, then one quick ramp to
# the stock limit by 0.6 s. Road test 2026-09-27 11:22 (stopped car ahead): the earlier 1.5 s build-up was soft at
# first but joined the normal brake too slowly and the rest of the stop felt uneven; the driver wants only the very
# first touch softened and the normal force picked up as soon as possible after it.
# 2026-10-01 (driver: "a prepared human touches lightly for the first 0.1 s, then presses linearly harder to whatever
# peak is needed - reach it as fast as possible but smoothly, never stomp, never drop it suddenly"; device campaign with
# the shaper modelled): every brake onset now gets the same shape at every speed - 1.0 m/s^3 for 0.1 s (about -0.1 at
# 0.1 s), then the jerk limit rises to the stock 4.0 m/s^3 by 0.3 s. The old low-speed schedule (0.25 -> 0.6 -> 4.0 over
# 0.3 / 1.2 s below 40 km/h) delayed a stop so much that the governor had to catch up and ended 6 m short at 20-30 km/h;
# with this shape the nominal stops land on 3.0 m and still end on the template.
HARD_BRAKE_ACCEL = -1.5  # m/s^2, requests below this are urgent ...
# ... and get a very short soft start instead of the stock limit straight away (driver 2026-09-30: the only stabs left
# were the MPC's hard requests, 2 of 28 onsets in the 2026-09-27 data, at ~37 km/h with the lead 29 m away and TTC ~4 s):
# URGENT_J for the first URGENT_T of the onset, then the stock limit. Costs ~0.05 m/s^2 of braking in that 0.1 s
# (< 10 cm of stopping distance at 40 km/h). FCW still bypasses everything.
URGENT_T = 0.1  # s
URGENT_J = 1.0  # m/s^3
# ... and the jerk limit then rises to the stock value over URGENT_T_RAMP instead of stepping, so the brake builds
# linearly after the light first touch (driver 2026-10-01: "a prepared human touches lightly, then presses linearly
# harder, up to -3.0 if needed - allow a higher peak but reach it smoothly")
URGENT_T_RAMP = 0.1  # s


class BrakeOnsetShaper:
  def __init__(self, dt: float, stock_down_jerk: float):
    self.dt = dt
    self.stock_down_jerk = stock_down_jerk
    self.t_onset = 0.0

  def reset(self) -> None:
    self.t_onset = 0.0

  @staticmethod
  def schedule_t(v_ego: float) -> list[float]:
    t1 = float(np.interp(v_ego, ONSET_V_BP, ONSET_T1_V))
    t3 = float(np.interp(v_ego, ONSET_V_BP, ONSET_T3_V))
    return [0.0, t1, t3]

  def down_step(self, accel_request: float, prev_accel: float, bypass: bool = False, v_ego: float = 30.0,
                urgent: bool = False) -> float:
    """Negative per-frame step allowed for the command this cycle (feed as dw_step to rate_limit).
    bypass (FCW): stock limit at once. urgent (hard request): URGENT_J for the first URGENT_T, then stock."""
    if bypass:
      self.t_onset = 0.0
      return -self.stock_down_jerk * self.dt

    if urgent:
      t_bp, j_bp = [0.0, URGENT_T, URGENT_T + max(URGENT_T_RAMP, self.dt)], [URGENT_J, URGENT_J, self.stock_down_jerk]
    else:
      t_bp, j_bp = self.schedule_t(v_ego), ONSET_J_DOWN
    gentlest_step = -ONSET_J_DOWN[0] * self.dt
    onset = (accel_request - prev_accel) < gentlest_step - 1e-9
    if onset:
      j_down = float(np.interp(self.t_onset, t_bp, j_bp))
      self.t_onset = min(self.t_onset + self.dt, t_bp[-1])
    else:
      # a gentle or rising request lets the schedule wind back at real time, so a short pause in a brake
      # does not fully re-arm it while a settled brake does
      j_down = self.stock_down_jerk
      self.t_onset = max(self.t_onset - self.dt, 0.0)
    return -min(j_down, self.stock_down_jerk) * self.dt

  def is_urgent(self, accel_request: float, fcw: bool) -> bool:
    return fcw or accel_request < HARD_BRAKE_ACCEL


# Engage onset: the same idea for the GAS side, but only at the moment longitudinal control engages (driver
# request 2026-09-27, Altis: "on engage, gas or brake, the first 0.1 s should be the lightest touch and then
# blend in"). The brake side already gets this from BrakeOnsetShaper (it re-arms whenever control was inactive).
# For the positive direction the upward jerk limit follows this schedule from the engage instant and is back to
# the stock 4 m/s^3 by 0.6 s; after that gas requests are unshaped so lead follow-away stays crisp.
ENGAGE_T_BP = [0.0, 0.1, 0.6]  # s since longitudinal control engaged
ENGAGE_J_UP = [0.25, 0.6, 4.0]  # m/s^3 upward jerk limit


class EngageOnsetShaper:
  def __init__(self, dt: float, stock_up_jerk: float):
    self.dt = dt
    self.stock_up_jerk = stock_up_jerk
    self.t_engaged: float | None = None

  def reset(self) -> None:
    self.t_engaged = None

  @property
  def in_engage_window(self) -> bool:
    """True from the engage instant until the schedule has reached stock (0.6 s): the brake side must keep its soft
    onset here even for a hard request, so a low-speed engage close behind a lead does not stab the brake
    (driver 2026-09-27). FCW still bypasses everything."""
    return self.t_engaged is None or self.t_engaged < ENGAGE_T_BP[-1]

  def up_step(self, active: bool) -> float:
    """Positive per-frame step allowed for the command this cycle (feed as up_step to rate_limit)."""
    if not active:
      self.t_engaged = None
      return self.stock_up_jerk * self.dt
    if self.t_engaged is None:
      self.t_engaged = 0.0
    else:
      self.t_engaged += self.dt
    if self.t_engaged >= ENGAGE_T_BP[-1]:
      return self.stock_up_jerk * self.dt
    j_up = float(np.interp(self.t_engaged, ENGAGE_T_BP, ENGAGE_J_UP))
    return min(j_up, self.stock_up_jerk) * self.dt


# Hybrid regen -> friction brake handover (Corolla Altis Hybrid). Tonight's 13 openpilot stops (routes 000000e6-ea,
# 2026-10-01/02), delivered IMU deceleration / request 0.3 s earlier, by speed: ~0.9 above 5.5 km/h and below 2.5 km/h,
# 1.01 at 4.5-5.5, 1.21 at 3.5-4.5, 1.12 at 2.5-3.5 km/h - the car bites ~20-35% harder exactly where the regen share
# hands over to the friction brakes, the "brakes, lets go, brakes again" bump just before the stop (22:49: delivered
# 0.13 at 5.0 km/h then 0.85 at 3.5 km/h for a steady ~0.7 request). The braking request is scaled down there so the
# delivered deceleration stays flat. Breakpoints are the speed at which the request is SENT; it is delivered ~0.3 s
# later, ~0.8 km/h slower. Driver 2026-10-02: "13 stops are enough data, change it".
HANDOVER_V_BP = [2.8 / 3.6, 3.8 / 3.6, 4.8 / 3.6, 5.8 / 3.6, 6.8 / 3.6]   # m/s
HANDOVER_SCALE = [1.0, 0.84, 0.78, 0.91, 1.0]


def handover_scale(v_ego: float, accel: float) -> float:
  """Braking request after the regen->friction handover compensation (positive requests unchanged)."""
  if accel >= 0.0:
    return accel
  return accel * float(np.interp(v_ego, HANDOVER_V_BP, HANDOVER_SCALE))
