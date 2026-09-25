"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lead start assist: when the car is stopped behind a stopped lead and the lead starts to move, the stock lead MPC
takes close to a second before it asks for enough acceleration to leave the stopping state. Its distance cost is
scaled by 1 / (v_ego + 10) and the jerk cost dominates, so a lead that has only moved half a metre produces an
acceleration target below the 0.1 m/s^2 that should_stop() needs, and the Toyota PCM keeps the brakes on until the
request passes 0.3 m/s^2. In a drive-through lane the lead often moves one car length and stops again, so the car
does not move at all.

This module watches for the lead departure directly (raw lead speed or gap growth since the stop) and, once it is
seen, puts a floor under the planner's acceleration target for a short time. The MPC keeps control of everything
above that floor and takes over again as soon as the car is rolling, the gap stops growing, or the lead is lost.
"""
import numpy as np

V_EGO_STOPPED = 0.3  # m/s, ego counts as stopped below this while arming
V_EGO_HANDOFF = 1.2  # m/s, once ego rolls faster than this the MPC has clearly taken over
LEAD_D_MIN = 2.0  # m, closest lead the assist will ever push towards (stops land at STOP_GAP = 3.0 m)
LEAD_D_MAX = 12.0  # m, farthest lead that counts as "the car we are queued behind"
LEAD_V_STATIONARY = 0.2  # m/s, lead speed below which it counts as stopped while arming
ARM_TIME = 0.1  # s, both cars stopped for this long before the assist is armed
LEAD_JUMP = 1.0  # m, a gap that closes this much in one frame is a different car (cut-in), start over
LEAD_V_GO = 0.15  # m/s, raw lead speed that counts as "it is moving"; radar noise at a stop is < 0.1 (p99 0.075),
                  # so this is as sensitive as the noise floor allows
LEAD_D_GO = 0.35  # m, gap growth since the stop that counts as "it is moving"
LEAD_V_CONFIRM = 0.12  # m/s, gap growth only counts as a departure when the lead speed also shows motion,
                       # so a stationary lead's radar drift or an ego roll-back can never relaunch the car
GO_CONFIRM_TIME = 0.25  # s, the departure signal must hold this long, so a 1-2 frame radar-speed spike on a
                        # STILL lead does not push into it and cause a stop-move-brake (Cross rlog 2026-09-21)
GAP_MIN_ACTIVE = 2.3  # m, release the floor if the gap gets this small
LEAD_CLOSING_V_REL = -0.25  # m/s, release the floor once the lead is clearly slower than ego (gap shrinking)
ACTIVE_T_BP = [0.0, 0.5]  # s since the departure was seen
ACTIVE_A_V = [0.45, 0.65]  # m/s^2 floor on the planner's acceleration target; the PCM took ~0.5 s to roll once the request passed 0.3
ACTIVE_TIMEOUT = 3.0  # s, hand back to the MPC no matter what after this long


class LeadStartAssist:
  def __init__(self, dt: float):
    self.dt = dt
    self.reset()

  def reset(self) -> None:
    self.t_stopped = 0.0
    self.armed = False
    self.d_stop = float('inf')
    self.t_lead_moving = 0.0
    self.t_gap_growing = 0.0
    self.d_prev: float | None = None
    self.t_active: float | None = None

  @property
  def active(self) -> bool:
    return self.t_active is not None

  def update(self, allowed: bool, v_ego: float, lead_present: bool, d_rel: float, v_lead: float, v_rel: float) -> float | None:
    """Returns the acceleration floor to apply this cycle, or None when the assist has nothing to say.

    v_rel follows radarState: positive when the lead is faster than ego."""
    lead_jumped = self.d_prev is not None and self.d_prev - d_rel > LEAD_JUMP
    if not allowed or not lead_present or not (LEAD_D_MIN <= d_rel <= LEAD_D_MAX) or lead_jumped:
      self.reset()
      self.d_prev = d_rel if lead_present else None
      return None
    self.d_prev = d_rel

    if self.t_active is not None:
      self.t_active += self.dt
      if (v_ego > V_EGO_HANDOFF or self.t_active > ACTIVE_TIMEOUT or
          d_rel < GAP_MIN_ACTIVE or v_rel < LEAD_CLOSING_V_REL or d_rel < self.d_stop - 0.3):
        self.reset()
        return None
      return float(np.interp(self.t_active, ACTIVE_T_BP, ACTIVE_A_V))

    if v_ego > V_EGO_HANDOFF:
      self.reset()
      return None

    if not self.armed:
      if v_ego < V_EGO_STOPPED and v_lead < LEAD_V_STATIONARY:
        self.t_stopped += self.dt
        self.d_stop = min(self.d_stop, d_rel)
        if self.t_stopped >= ARM_TIME - 1e-6:
          self.armed = True
      else:
        self.t_stopped = 0.0
        self.d_stop = float('inf')
      return None

    # armed: both cars have been stopped together, watch for the lead to leave
    self.d_stop = min(self.d_stop, d_rel)
    self.t_lead_moving = self.t_lead_moving + self.dt if v_lead >= LEAD_V_GO else 0.0
    gap_departure = (d_rel - self.d_stop >= LEAD_D_GO) and (v_lead >= LEAD_V_CONFIRM)
    self.t_gap_growing = self.t_gap_growing + self.dt if gap_departure else 0.0
    if max(self.t_lead_moving, self.t_gap_growing) >= GO_CONFIRM_TIME - 1e-6:
      self.t_active = 0.0
      return ACTIVE_A_V[0]
    return None
