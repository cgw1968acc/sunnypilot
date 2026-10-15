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
END_ACCEL = -0.34      # m/s^2 at 0 km/h. tncr (Corolla Cross): -0.35 -> -0.34 (owner 2026-10-15 evening, "0.01 lighter");
                       # -0.35 was the Cross stop template (stopped at
                       # 800-840 N, no nod); the Altis rule above is -0.30 and stays so on tnpb3
HOLD_RATE = 0.5        # m/s^3: after the stop, toward the standstill hold
# tncr (Corolla Cross), owner 2026-10-15 evening after route 00000073 17:16:08 (a stop that ended with 4120 N at
# 2.7 km/h and -1.67 m/s^2 still on the car as the wheels stopped - the jolt): 'the car must, without exception, stop
# on a force that cannot jolt; the end must ease smoothly down to -0.34 at 0 km/h; 5 km/h may still need a firm brake'.
# So besides the anchored line below V_LINE there is a FIXED ceiling line from CEIL_V down to END_ACCEL at 0 km/h that
# nothing may exceed while the car moves (the roll guard included - controlsd no longer bypasses this stage): -2.0 at
# 5 km/h, -1.34 at 3, -1.0 at 2, -0.67 at 1, -0.34 at 0. An ordinary stop (END curve -0.65 at 5 km/h) never touches
# it; only the hold / guard / boost extras are capped. From 5 km/h at -2.0 (~2800 N) the pressure has to come off at
# ~2000 N/s to follow the line; the Cross released 4120 -> 1400 N in 0.4 s on 17:16:07 (~6800 N/s), so it can.
CEIL_V = 5.0 / 3.6     # m/s
CEIL_TOP = -2.0        # m/s^2 at CEIL_V
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

    ceil_in = accel
    if v_ego <= CEIL_V:  # the fixed ceiling line (never firmer, whatever asked for it)
      ceil = END_ACCEL + (CEIL_TOP - END_ACCEL) * max(v_ego, 0.0) / CEIL_V
      if accel < ceil:
        accel = ceil
    if v_ego >= V_LINE:
      self.a_line, self.v_line, self.out, self.stall_t = accel, V_LINE, None, 0.0
      self.out = accel if accel > ceil_in else None
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
    self.out = accel if accel > ceil_in else None
    return accel
