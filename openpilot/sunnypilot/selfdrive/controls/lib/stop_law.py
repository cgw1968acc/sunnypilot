"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Stop law (owner 2026-10-11): "whatever the distance, work out the braking distance, press the brake from the lightest touch
and build it smoothly, peak before half the distance, then ease it off smoothly and stop at the lightest force - no jolt at
the stop. When the distance changes, recompute with the same rule and join the current force smoothly. The peak may go past
-3 if it is reached smoothly. In an emergency the stop may end 2.5 m behind the lead instead of 3.5 m."

Studied on every stop behind a lead in the C3X logs (claude_work/brake_profile: 1669 qlog segments, 185 openpilot stops
from >= 10 km/h, closed-loop replay with the recorded lead and a 0.3 s actuator lag). Pre-planned time profiles
(build -> peak -> release, re-planned every 0.1 s) failed closed loop: each re-plan pushed the peak and the release later
and the end got squeezed. A state-feedback law does what the owner described by itself:
  b* = max(B_END, G * v^2 / (2 D))     D = distance to the stop point (lead's stop point - GAP_CMD - ego travel)
G > 1 brakes a little more than the constant decel that would just make it, so the needed decel then keeps FALLING and the
command follows it down (b ~ D^(G-1)): the release and the light end come from the physics, and a changing D never leaves
a seam because nothing is planned ahead. The command tracks b* with a time constant TAU, its jerk capped (J_BUILD up,
J_RELEASE down) and its jerk change capped (SNAP), so every build starts from zero jerk (S-shaped, from the lightest touch).
Replay results vs the recorded stops (median / p90 / max):
  build jerk     2.31 / 3.76 / 6.23 -> 1.42 / 1.75 / 3.00 m/s^3  (> 2.5: 74 stops -> 1)
  peak decel     1.80 / 2.78 / 4.26 -> 1.53 / 2.14 / 3.78 m/s^2
  decel at stop  0.76 / 0.93 / 1.74 -> 0.30 / 0.33 / 2.57 m/s^2  (> 0.6: 183 stops -> 7)
  peak position  63% -> 39% of the braking distance; stop gap median 3.4 -> 3.7 m
  route 00000133 15:04 (48 km/h, lead braking -2.2): peak -2.93 -> -2.17, build 3.2 -> 1.4, stop decel 1.26 -> 0.31
  route 00000134 15:29 (60 km/h, stopped car found at 51 m): peak -4.08 -> -3.76, build 4.8 -> 3.0, stop 0.91 -> 0.33,
  2.6 m behind (2.5 m mode)
The law only takes over while the lead is STOPPING (braking, or below LEAD_SLOW_V); following a moving lead stays with the
MPC. When the lead drives on (or is lost) the output goes back to the planner's own target at HANDBACK_JERK. FCW, e2e and
the driver's pedals keep the stock path. Off with `touch /data/tn_no_stop_law` (checked every second, no restart).
"""
import os

import numpy as np

from openpilot.common.swaglog import cloudlog

G = 1.3  # brake this much more than the constant decel that would just stop at the target
B_END = 0.30  # m/s^2: the lightest force, held to the stop (owner: -0.30 "closer to perfect")
COAST = 0.10  # m/s^2: target while far more room than needed (above END_V)
END_V = 1.0  # m/s: below this the target never drops under B_END
GAP_CMD = 3.3  # m commanded; the replay lands at 3.5-3.7 m (lag + the B_END crawl)
GAP_CMD_EMERG = 2.3  # m: when 3.5 m would need more than A_SOFT (owner: 2.5 m then), lands ~2.5 m
A_SOFT = 3.5  # m/s^2
LEAD_DECEL_MIN = 1.5  # m/s^2: the lead's stop point assumes it brakes at least this hard
STOP_PT_TAU = 0.4  # s: the stop point moves AWAY this slowly (radar noise); it moves closer at once
STOP_PT_SNAP = 0.5  # m: a move closer larger than this is taken at once
TAU = 0.45  # s: command time constant toward b*
J_BUILD = 2.0  # m/s^3 (x2 in the 2.5 m mode)
J_RELEASE = 1.5  # m/s^3
SNAP = 8.0  # m/s^4 (x3 in the 2.5 m mode or in danger)
V_MAX = 75.0 / 3.6  # m/s: stops from city speeds only; the highway stays with the MPC
LEAD_STOPPING_A = -0.5  # m/s^2: the lead counts as stopping when braking harder than this ...
LEAD_SLOW_V = 2.0  # m/s: ... or slower than this
LEAD_GO_A = -0.2  # m/s^2: and drives on again when it brakes less than this ...
LEAD_GO_V = 3.0  # m/s: ... and is faster than this
HANDBACK_JERK = 1.5  # m/s^3: from the law's output back to the planner's target
DANGER_DECEL = 4.0  # m/s^2: danger = stopping at this decel would leave less than DANGER_MARGIN
DANGER_MARGIN = 1.0  # m
DISABLE_FLAG = "/data/tn_no_stop_law"
FLAG_CHECK_PERIOD = 1.0  # s


class StopLaw:
  def __init__(self, dt: float):
    self.dt = dt
    self.enabled = True
    self._flag_t = FLAG_CHECK_PERIOD
    self.reset()

  def reset(self) -> None:
    self.active = False
    self.handback = False
    self.out = 0.0
    self.b = 0.0
    self.j = 0.0
    self.s = 0.0
    self.stop_pt = None
    self.gap = GAP_CMD

  def _check_flag(self) -> None:
    self._flag_t += self.dt
    if self._flag_t >= FLAG_CHECK_PERIOD:
      self._flag_t = 0.0
      self.enabled = not os.path.exists(DISABLE_FLAG)

  def _end(self, reason: str, a_plan: float):
    if self.active:
      cloudlog.event("stop_law: hand back", reason=reason, out=round(self.out, 2), error=True)
    self.active, self.handback = False, True
    return self._handback(a_plan)

  def _handback(self, a_plan: float):
    step = HANDBACK_JERK * self.dt
    if abs(a_plan - self.out) <= step:
      self.reset()
      return None
    self.out += float(np.clip(a_plan - self.out, -step, step))
    return self.out

  def update(self, allowed: bool, v_ego: float, standstill: bool, lead_present: bool, d_rel: float, v_lead: float,
             a_lead: float, a_plan: float, a_prev: float):
    """a_plan: the planner's target this cycle; a_prev: last cycle's output. Returns the target to use, or None (stock)."""
    self._check_flag()
    if not self.enabled or not allowed or standstill:
      self.reset()
      return None
    if self.handback:
      return self._handback(a_plan)

    lead_stopping = lead_present and (a_lead < LEAD_STOPPING_A or v_lead < LEAD_SLOW_V)
    if not self.active:
      if not lead_stopping or v_ego > V_MAX or v_ego < 0.5:
        return None
      self.active, self.s, self.stop_pt, self.gap = True, 0.0, None, GAP_CMD
      self.b, self.j = max(0.0, -a_prev), 0.0
      cloudlog.event("stop_law: take over", v=round(v_ego * 3.6, 1), d=round(d_rel, 1), v_lead=round(v_lead * 3.6, 1),
                     a_lead=round(a_lead, 2), a_plan=round(a_plan, 2), error=True)
    elif not lead_present:
      return self._end("lead lost", a_plan)
    elif a_lead > LEAD_GO_A and v_lead > LEAD_GO_V:
      return self._end("lead drives on", a_plan)
    elif v_ego > V_MAX + 1.0:
      return self._end("too fast", a_plan)

    # the lead's stop point, in metres from where the law took over
    self.s += v_ego * self.dt
    v_l = max(v_lead, 0.0)
    stop_pt = self.s + d_rel + v_l * v_l / (2.0 * max(-a_lead, LEAD_DECEL_MIN))
    if self.stop_pt is None or stop_pt < self.stop_pt - STOP_PT_SNAP:
      self.stop_pt = stop_pt
    else:
      self.stop_pt += (stop_pt - self.stop_pt) * self.dt / (STOP_PT_TAU + self.dt)

    D = self.stop_pt - self.gap - self.s
    r = v_ego * v_ego / (2.0 * max(D, 0.05))
    if self.gap == GAP_CMD and G * r > A_SOFT and v_ego > 1.0:
      self.gap = GAP_CMD_EMERG
      cloudlog.event("stop_law: 2.5 m mode", v=round(v_ego * 3.6, 1), d=round(d_rel, 1), need=round(G * r, 2), error=True)
      D = self.stop_pt - self.gap - self.s
      r = v_ego * v_ego / (2.0 * max(D, 0.05))
    emerg = self.gap != GAP_CMD
    danger = d_rel - v_ego * v_ego / (2.0 * DANGER_DECEL) < DANGER_MARGIN

    floor = B_END if (v_ego < END_V or r > B_END / G) else COAST
    b_star = max(floor, G * r)
    jb = 20.0 if danger else J_BUILD * (2.0 if emerg else 1.0)
    jr = 20.0 if danger else J_RELEASE
    snap = SNAP * (3.0 if danger or emerg else 1.0)
    j_tgt = float(np.clip((b_star - self.b) / TAU, -jr, jb))
    self.j = float(np.clip(j_tgt, self.j - snap * self.dt, self.j + snap * self.dt))
    self.b = max(0.0, self.b + self.j * self.dt)
    self.out = -self.b
    if danger and a_plan < self.out:  # in danger never lighter than the planner
      self.out, self.b = a_plan, -a_plan
    return self.out
