"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.realtime import DT_CTRL

# End-force limit - the owner's end-of-stop rule (2026-10-14 evening, "remember it forever"): the request at the 0 km/h
# stop is always -0.30 (or -0.28), never firmer; a safe, effective stop does all its force modulation ABOVE 2 km/h;
# "if the MPC asks for more than this, always override it". Evidence: route 0000015a 18:51:26 (engaged at 7 km/h behind
# a stopped lead 4 m ahead) stopped at -0.46 with the car at -1.5..-1.9 m/s^2 - "the last moment too strong"; route
# 0000015c 19:35:40 stopped at -0.34.
# So this is the LAST stage of the request (after the stop-gap hold/close, creep-stall guard and boost): below V_LINE the
# request may not be firmer than a line in speed from what it was at V_LINE (a_line) down to END_ACCEL at 0 km/h.
# Lighter is always passed. No exceptions. Once the wheels stop, the request continues from where the line left it and
# may only firm up at HOLD_RATE into whatever holds the car (the standstill hold), so the stop itself has no step.
# Safety valve: if below V_LINE the car is not slowing (aEgo above STALL_A for STALL_T while still rolling above
# STALL_V - the hybrid's creep torque can beat a light request), the line is let go and re-anchored at the current speed
# and request once it slows again: a stall is not "the stop", and it still ends on -0.30 at 0 km/h.
# Re-stop (owner 2026-10-18: "stopped, crept forward, stopped again - the force can still be a little lighter; only
# this case -0.30 -> -0.26"): once the car has stood RESTOP_STAND_T, a stop made within RESTOP_WINDOW of moving off
# again without reaching RESTOP_V_MAX (tighter than the stopping controller's creep-follow window) ends on
# RESTOP_END_ACCEL instead - only behind a lead (owner: "only with a lead, not in any other normal stop"): a lead
# within RESTOP_LEAD_D must have been seen since moving off (latched, so a flickering lead cannot firm the line back up).
# Every other stop, with or without a lead, keeps END_ACCEL.
V_LINE = 2.0 / 3.6     # m/s
END_ACCEL = -0.30      # m/s^2 at 0 km/h
HOLD_RATE = 0.5        # m/s^3: after the stop, toward the standstill hold
STALL_V = 0.5 / 3.6    # m/s
STALL_A = -0.1         # m/s^2
STALL_T = 0.3          # s
RESTOP_END_ACCEL = -0.26  # m/s^2 at 0 km/h for a re-stop after a creep
RESTOP_STAND_T = 0.5      # s standing that arms it
RESTOP_WINDOW = 5.0       # s since moving off (owner: no longer than 5 s)
RESTOP_V_MAX = 5.0 / 3.6  # m/s (owner: the creep never above 5 km/h)
RESTOP_LEAD_D = 8.0       # m: a lead this close counts (creeping up behind it; owner: within 8 m)
RESTOP_CHECK_V = 1.0 / 3.6  # m/s: the 5 s limit is checked down to here (owner: from 1 km/h, not 2)


class EndForceLimit:
  def __init__(self):
    self.reset()
    self.stand_t = 0.0
    self.restop_t = None  # time since moving off after a standstill (None = not a re-stop)
    self.restop_vmax = 0.0
    self.restop_lead = False
    self.restop_on = False

  def _update_restop(self, v_ego, standstill, long_active, lead_d):
    if not long_active:
      self.stand_t, self.restop_t = 0.0, None
    elif standstill:
      self.stand_t += DT_CTRL
      if self.stand_t >= RESTOP_STAND_T:
        self.restop_t, self.restop_vmax, self.restop_lead = 0.0, 0.0, False
    else:
      self.stand_t = 0.0
      if self.restop_t is not None:
        self.restop_t += DT_CTRL
        self.restop_vmax = max(self.restop_vmax, v_ego)
        # the time limit is checked down to RESTOP_CHECK_V; below it the end is decided, so a slow stop can never
        # switch to the firmer -0.30 in its last moment (item 0)
        if self.restop_vmax >= RESTOP_V_MAX or (self.restop_t > RESTOP_WINDOW and v_ego >= RESTOP_CHECK_V):
          self.restop_t = None
    if self.restop_t is not None and lead_d is not None and lead_d <= RESTOP_LEAD_D:
      self.restop_lead = True
    return self.restop_t is not None and self.restop_lead

  def reset(self):
    self.a_line = None   # request at the line's anchor (crossing V_LINE, or the end of a stall)
    self.v_line = V_LINE # speed at the anchor
    self.out = None      # last limited output (None = not limiting)
    self.stall_t = 0.0

  def update(self, accel: float, v_ego: float, standstill: bool, long_active: bool, a_ego: float = -1.0,
             lead_d: float | None = None) -> float:
    # lead_d: distance to the lead in metres, None without a lead
    restop = self._update_restop(v_ego, standstill, long_active, lead_d)
    end_accel = RESTOP_END_ACCEL if restop else END_ACCEL
    if self.restop_on and not restop and not standstill and v_ego < V_LINE and self.a_line is not None:
      # the re-stop ran out between 1 and 2 km/h: the -0.30 line starts from the current command and speed, no step
      self.a_line, self.v_line = (self.out if self.out is not None else accel), max(v_ego, STALL_V)
    self.restop_on = restop
    if not long_active or accel >= 0.0:
      self.reset()
      return accel

    if standstill:
      if self.out is None:
        return accel
      self.out = max(accel, self.out - HOLD_RATE * DT_CTRL)
      if self.out <= accel + 1e-6:
        self.out = None
        return accel
      return self.out

    if v_ego >= V_LINE:
      self.a_line, self.v_line, self.out, self.stall_t = accel, V_LINE, None, 0.0
      return accel

    self.stall_t = self.stall_t + DT_CTRL if (v_ego > STALL_V and a_ego > STALL_A) else 0.0
    if self.a_line is None or self.stall_t >= STALL_T:  # braking started below the line, or a stall: anchor here
      self.a_line, self.v_line = accel, max(v_ego, STALL_V)
      if self.stall_t >= STALL_T:
        self.out = None
        return accel
    a_top = min(self.a_line, end_accel)
    floor = end_accel + (a_top - end_accel) * min(max(v_ego, 0.0) / self.v_line, 1.0)
    if accel < floor:
      self.out = floor
      return floor
    self.out = None
    return accel
