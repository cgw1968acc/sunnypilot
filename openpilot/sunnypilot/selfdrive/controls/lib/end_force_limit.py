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
V_LINE = 2.0 / 3.6     # m/s
END_ACCEL = -0.35      # m/s^2 at 0 km/h. tncr (Corolla Cross, owner 2026-10-15): -0.35, the Cross stop template (stopped at
                       # 800-840 N, no nod); the Altis rule above is -0.30 and stays so on tnpb3
HOLD_RATE = 0.5        # m/s^3: after the stop, toward the standstill hold
STALL_V = 0.5 / 3.6    # m/s
STALL_A = -0.1         # m/s^2
STALL_T = 0.3          # s


class EndForceLimit:
  def __init__(self):
    self.reset()

  def reset(self):
    self.a_line = None   # request at the line's anchor (crossing V_LINE, or the end of a stall)
    self.v_line = V_LINE # speed at the anchor
    self.out = None      # last limited output (None = not limiting)
    self.stall_t = 0.0

  def update(self, accel: float, v_ego: float, standstill: bool, long_active: bool, a_ego: float = -1.0) -> float:
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
    a_top = min(self.a_line, END_ACCEL)
    floor = END_ACCEL + (a_top - END_ACCEL) * min(max(v_ego, 0.0) / self.v_line, 1.0)
    if accel < floor:
      self.out = floor
      return floor
    self.out = None
    return accel
