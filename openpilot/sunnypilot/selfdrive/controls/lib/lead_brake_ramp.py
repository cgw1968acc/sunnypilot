"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.realtime import DT_CTRL

# Progressive brake behind a braking lead (tncr, owner 2026-10-22). After halving the dynamic close-follow gap the owner
# asked: "a human stops smoothly from a shorter distance - why can't the C4? With the shorter gap, do not stamp on the
# brake: after the soft first touch keep building the force a little more, and stop smoothly."
# Why the C4 brakes late and hard: on route 00000084 seg 8 (lead -3 m/s^2 at 41 km/h) the radar showed the lead's speed
# drop 0.15 s after it started braking and aLeadK passed -0.8 at 0.30 s, but our command reached -0.3 only at 0.75 s and
# the car decelerated at 0.95 s; by then the lead is much closer and the planner (which also insists on keeping the
# 1.45 s follow time while decelerating) has to ask for -3.1. A human starts within that 0.45 s and lets the gap shrink.
# What it does: once the lead brakes (aLeadK <= ARM_A) and we are above V_ON it computes every frame the constant decel
# that stops us GAP behind the point where the lead itself stops (lead stop distance vLead^2 / 2|aLeadK|, decel floored
# at LEAD_A_MIN) - the stop_gap_close law, recomputed from the measured state, a feedback, not a table. A ramp follows
# that need: J_SOFT for the first T_SOFT s (the touch), then a steady J_ON, never firmer than the need; while the need
# gets lighter it eases at J_OFF only. The request is the firmer of the chain's and the ramp's, and a planner request
# firmer than the need x CAP_K + CAP_ADD is held back to that (the need already stops us GAP behind the lead and is
# recomputed every frame, so it firms up by itself if the lead brakes harder). Below V_FADE it hands back to the stop
# chain by 6 km/h (stopping controller, stop-gap close/hold and the end ceiling, all after or unchanged); the end
# ceiling still comes last. It lets go at J_REL when a moving lead stops braking (a brake tap: aLeadK > DISARM_A while
# vLead > LEAD_STOPPED_V), pulls away, or is lost. ENABLED = False turns it off.
# Closed-loop sim (x86 build of the real long MPC + LongControl + StoppingController + the whole shaping chain + the
# Toyota brake-onset shaper, car = 0.15 s dead time + 0.3 s lag + creep model), standard follow, now -> with it:
#   45 km/h lead -3 to stop: peak -3.11 -> -2.80, stop 2.65 -> 2.52 m    60 km/h lead -3: -3.29 -> -2.92, 2.61 -> 2.54 m
#   45 km/h lead -4:         -3.50 -> -3.50, stop 2.28 -> 2.75 m          30 km/h lead -2.5: -2.47 -> -2.16, 3.12 -> 2.88 m
#   45 km/h lead -2:         -2.23 -> -2.03, stop 3.71 -> 3.39 m          25 km/h lead -2:   -2.15 -> -1.74, 4.03 -> 3.34 m
#   50 km/h lead -1.5:       -1.78 -> -1.63, stop 3.73 -> 3.35 m
#   lead slows 45 -> 25 km/h at -1.5: -1.66 with a 1.87 release-and-brake-again -> -1.34 in one build
#   lead brake taps (-1.2 for 0.6 s at 45, -1.0 for 1 s at 60): command -0.20/-0.28 -> -0.41/-0.54 briefly, lowest speed
#   unchanged (42.5 / 56.1 km/h); no case makes the brake firmer again below 10 km/h, the 1-3 km/h end is unchanged.
ENABLED = True
ARM_A = -0.8               # m/s^2: lead decel that arms it (same as the dynamic gap's DYN_A0)
DISARM_A = -0.3            # m/s^2: a moving lead no longer braking harder than this lets go (a brake tap)
LEAD_STOPPED_V = 2.0       # m/s: ... but a lead (nearly) stopped keeps it (its aLeadK decays to 0 once it stands)
LEAD_A_MIN = 0.8           # m/s^2: floor of the lead decel used for where the lead stops
GAP = 3.0                  # m: aim (the stop chain then settles it to 3-3.5 m)
B_MIN = 0.3                # m/s^2: a lighter need is ignored (no nibbling)
J_SOFT = 1.5               # m/s^3: first touch ...
T_SOFT = 0.1               # s: ... for this long
J_ON = 2.5                 # m/s^3: then the steady build
J_OFF = 0.5                # m/s^3: easing while armed when the need gets lighter
J_REL = 1.5                # m/s^3: letting go once disarmed
V_ON = 20.0 / 3.6          # m/s: arms only above this
V_FADE = [6.0 / 3.6, 10.0 / 3.6]  # m/s: hands the stop to the stop chain
CAP = True                 # also hold a firmer plan back to the need (x CAP_K + CAP_ADD)
CAP_K = 1.15
CAP_ADD = 0.2              # m/s^2
# The cap is a level that moves at limited rates, never a switch (route 0000009e 20:11:04, true time: when the lead
# stopped braking the ramp disarmed and the open-loop replay showed the request stepping 0.15 firmer, -0.73 -> -0.88,
# the instant the cap vanished). It starts at the request it meets (no step when it arms), follows a firmer need at
# CAP_J_FIRM, a lighter one at CAP_J_LIGHT, and once disarmed (or the need is gone) it opens towards "no cap" at J_REL.
CAP_J_FIRM = 3.0           # m/s^3
CAP_J_LIGHT = 1.0          # m/s^3
CAP_OFF_A = -3.0           # m/s^2: a request this firm (an emergency) is never held back


def brake_need(v_ego: float, d_rel: float, v_lead: float, a_lead: float) -> float:
  """constant decel (> 0) that stops us GAP behind where the lead stops"""
  lead_stop = v_lead ** 2 / (2.0 * max(-a_lead, LEAD_A_MIN)) if v_lead > 0.0 else 0.0
  room = max(d_rel + lead_stop - GAP, 0.5)
  return v_ego ** 2 / (2.0 * room)


class LeadBrakeRamp:
  def __init__(self):
    self.armed = False
    self.ramp = 0.0       # m/s^2 (<= 0)
    self.t_build = 0.0
    self.cap = None       # m/s^2: the cap level (the request is never firmer), None = no cap

  def reset(self):
    self.armed = False
    self.ramp = 0.0
    self.t_build = 0.0
    self.cap = None

  def update(self, accel: float, v_ego: float, standstill: bool, lead, long_active: bool, accel_min: float,
             hold_off: bool = False) -> float:
    if not ENABLED or not long_active or standstill:
      self.reset()
      return accel
    present = bool(lead.present)
    if not present or lead.vLead > v_ego + 1.0 or (lead.aLeadK > DISARM_A and lead.vLead > LEAD_STOPPED_V):
      self.armed = False
    elif lead.aLeadK <= ARM_A and v_ego >= V_ON:
      self.armed = True

    target = 0.0
    b = 0.0
    if self.armed and present:
      b = brake_need(v_ego, lead.dRel, lead.vLead, lead.aLeadK)
      if b >= B_MIN:
        target = -b
    if target < self.ramp:
      j = J_SOFT if self.t_build < T_SOFT else J_ON
      self.ramp = max(self.ramp - j * DT_CTRL, target)
      self.t_build += DT_CTRL
    else:
      self.ramp = min(self.ramp + (J_OFF if self.armed else J_REL) * DT_CTRL, target)
      if self.ramp >= 0.0:
        self.t_build = 0.0

    w = float(np.interp(v_ego, V_FADE, [0.0, 1.0]))
    if hold_off or w <= 0.0:
      return accel
    # only a braking ramp may firm the request: ramp == 0 must never touch it (7e45d71/94d2370 took min(accel, 0) and
    # capped every acceleration above 6 km/h at 0 - route 000000a1 20:54:28-32, plan +0.6..+1.7, sent 0.00 at 10 km/h)
    shaped = min(accel, self.ramp) if self.ramp < 0.0 else accel
    self._update_cap(shaped, b, accel <= CAP_OFF_A)
    if self.cap is not None:
      shaped = max(shaped, self.cap)
    if shaped == accel:
      return accel
    return max(accel + w * (shaped - accel), accel_min)

  def _update_cap(self, shaped: float, b: float, emergency: bool) -> None:
    if emergency:
      self.cap = None
    elif CAP and self.armed and b >= B_MIN:
      target = -(b * CAP_K + CAP_ADD)
      if self.cap is None:
        self.cap = min(shaped, target)  # meets the request where it is: no step
      elif target < self.cap:
        self.cap = max(self.cap - CAP_J_FIRM * DT_CTRL, target)
      else:
        self.cap = min(self.cap + CAP_J_LIGHT * DT_CTRL, target)
    elif self.cap is not None:
      self.cap -= J_REL * DT_CTRL
      if self.cap <= shaped:
        self.cap = None
