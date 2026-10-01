"""
Lead need cap: behind a MOVING lead, never brake much harder than the closing situation needs.

Driver's manual stops (C3X 09-27 backup, IMU): the delivered deceleration follows the kinematic need almost exactly at
every instant - closing speed^2 / (2 x remaining distance), plus a little more while the lead itself brakes (11:32:29
0.74/0.92 1.00/0.94 0.93/0.91 0.89/0.89 0.78/0.77; 13:38:31 1.91/1.92 3.00/2.62 3.33/3.08 3.40/3.29). Closing speed
alone explains 24% of the driver's force, the need (speed difference AND distance) 55%. The MPC brakes well above
the need when a lead only slows down (09-30..10-01: 1.5-3.0 for a need of 0.1-0.5). Driver 2026-10-01: "where the
brake should get lighter, make it lighter; when the MPC over-intervenes, find a fix."

The need is the constant deceleration that
  - for a lead braking (aLeadK < -0.3, blended in to -0.8): stops ego STOP_D behind where the lead will stop
  - for a steady lead: brings ego down to the lead's speed at STOP_D + FOLLOW_T x v_lead behind it
The MPC's request is capped at NEED_MARGIN x need when it is harder than that, only outside an emergency
(need <= NEED_MAX, time-to-collision >= TTC_MIN), and the cap is applied by easing at RELEASE_RATE, never as a step.
If the lead brakes harder, the need (and the allowed brake) grows at once. Leads slower than LEAD_V_MIN are left to
the stop-gap governor.
"""
import numpy as np

STOP_D = 3.5          # m, where a stop ends behind the lead
FOLLOW_T = 1.0        # s, headway the cap aims at for a steady lead (the MPC's 1.25 s is restored gently afterwards)
NEED_MARGIN = 1.15
NEED_MAX = 2.0        # m/s^2: above this it is an emergency, the MPC is left alone
TTC_MIN = 2.5         # s
LEAD_V_MIN = 2.0      # m/s: slower leads are the stop-gap governor's
EGO_V_MIN = 3.0       # m/s
LEAD_BRAKE_LO, LEAD_BRAKE_HI, LEAD_BRAKE_MAX = 0.3, 0.8, 3.0   # m/s^2
LEAD_A_TAU = 0.3      # s, low-pass on aLeadK
RELEASE_RATE = 1.0    # m/s^3, how fast a capped brake is eased (same as the stop governor's recovery)
GAP_MIN = 0.5         # m of room beyond the target, below which nothing is capped


def lead_need(v_ego: float, d_rel: float, v_lead: float, a_lead: float) -> float | None:
  """Constant deceleration (positive) the situation needs, or None when there is no room left to reason about."""
  vl = max(v_lead, 0.0)
  brake = max(-a_lead, 0.0)
  w = float(np.clip((brake - LEAD_BRAKE_LO) / (LEAD_BRAKE_HI - LEAD_BRAKE_LO), 0.0, 1.0))
  # steady lead: match its speed at the follow distance
  room_f = d_rel - (STOP_D + FOLLOW_T * vl)
  if room_f <= GAP_MIN and w < 1.0:
    return None
  need_f = (max(v_ego - vl, 0.0) ** 2) / (2.0 * max(room_f, GAP_MIN))
  # braking lead: stop behind where it will stop
  room_s = d_rel + vl ** 2 / (2.0 * min(max(brake, LEAD_BRAKE_LO), LEAD_BRAKE_MAX)) - STOP_D
  if room_s <= GAP_MIN:
    return None
  need_s = v_ego ** 2 / (2.0 * room_s)
  return (1.0 - w) * need_f + w * need_s


class LeadNeedCap:
  def __init__(self, dt: float):
    self.dt = dt
    self.reset()

  def reset(self) -> None:
    self.a_prev: float | None = None
    self.a_lead_f: float | None = None
    self.active = False

  def update(self, allowed: bool, v_ego: float, lead_present: bool, d_rel: float, v_lead: float, a_lead: float,
             a_mpc: float) -> float:
    a_mpc = float(a_mpc)
    if not allowed or not lead_present:
      self.reset()
      return a_mpc
    self.a_lead_f = float(a_lead) if self.a_lead_f is None else \
      self.a_lead_f + (float(a_lead) - self.a_lead_f) * min(self.dt / LEAD_A_TAU, 1.0)
    a_lead_used = min(self.a_lead_f, float(a_lead))   # react to a harder-braking lead at once, ease off filtered
    a_out = a_mpc
    self.active = False
    closing = v_ego - max(v_lead, 0.0)
    ttc = d_rel / closing if closing > 0.1 else np.inf
    if v_ego > EGO_V_MIN and v_lead >= LEAD_V_MIN and ttc >= TTC_MIN and a_mpc < 0.0:
      need = lead_need(v_ego, d_rel, v_lead, a_lead_used)
      if need is not None and need <= NEED_MAX:
        cap = -NEED_MARGIN * need
        if a_mpc < cap:
          a_out = cap
          self.active = True
    if self.active and self.a_prev is not None:
      a_out = min(a_out, self.a_prev + RELEASE_RATE * self.dt)   # ease into the cap, never a step
    a_out = max(a_out, a_mpc)                                       # never brake harder than the MPC
    self.a_prev = a_out
    return a_out
