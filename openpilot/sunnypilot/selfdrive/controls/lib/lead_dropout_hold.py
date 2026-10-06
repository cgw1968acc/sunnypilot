"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from typing import Any

# Lead drop-out hold (owner 2026-10-07). Route 0000010d 23:55:47-23:55:51 (C3X, Altis): following a lead that was
# braking from 52 km/h to a stop, the radar track behind the camera lead dropped out several times for 0.2-0.6 s and
# the lead became a vision-only target 42-49 m away "not braking" (aLeadK ~0) - the radar had it at 35-38 m braking at
# -1.2..-1.4. The plan eased from -1.2 to -0.8; when the radar track came back at 33 m with the lead braking at
# -2.2..-3.3, the car had to press to -2.7 m/s^2 at 34-30 km/h ("two-stage stop, a hard press at 30 km/h").
# Rule: remember the last RADAR lead within DROPOUT_MAX_D that was braking. If within DROPOUT_HOLD_T after it the lead
# becomes a vision-only target more than DROPOUT_JUMP_M farther than where that radar lead should be now (or there is
# no lead), keep the radar lead instead, moved on with its own speed and acceleration. A radar lead, or any lead closer
# than the held one, is used at once; after DROPOUT_HOLD_T without radar the vision lead is used.
DROPOUT_JUMP_M = 2.5  # m farther than the predicted radar lead counts as a drop-out
DROPOUT_HOLD_T = 1.0  # s, longest hold after the last radar sighting
DROPOUT_BRAKING_A = -0.5  # m/s^2: the radar lead was braking at least this much
DROPOUT_MAX_D = 60.0  # m: only a radar lead this close is kept
# (an earlier draft also kept a lead that was merely being closed on: over routes 00000100/00000109/0000010d that
# held 187/20/51 times, mostly far leads at 70-100 m - braking-only keeps it to the case it is meant for)


class LeadDropoutHold:
  def __init__(self, dt: float):
    self.dt = dt
    self.radar_lead: dict[str, Any] | None = None  # last radar lead that qualified, moved on every frame
    self.age = 0.0  # s since that radar lead was last actually seen

  def _predict(self, lead: dict[str, Any], v_ego: float) -> dict[str, Any]:
    p = dict(lead)
    v_lead = max(p['vLead'] + p['aLeadK'] * self.dt, 0.0)
    a_lead = p['aLeadK'] if v_lead > 0.0 else 0.0
    p['dRel'] = p['dRel'] + (p['vLead'] - v_ego) * self.dt
    p['vLead'] = p['vLeadK'] = v_lead
    p['aLeadK'] = a_lead
    p['vRel'] = v_lead - v_ego
    return p

  @staticmethod
  def _qualifies(lead: dict[str, Any]) -> bool:
    return lead['aLeadK'] < DROPOUT_BRAKING_A and lead['dRel'] < DROPOUT_MAX_D

  def update(self, lead: dict[str, Any], v_ego: float) -> dict[str, Any]:
    present = lead.get('present', False)
    if present and lead.get('radar', False):
      self.radar_lead = dict(lead) if self._qualifies(lead) else None
      self.age = 0.0
      return lead

    if self.radar_lead is None:
      return lead
    self.radar_lead = self._predict(self.radar_lead, v_ego)
    self.age += self.dt
    held = self.radar_lead
    if self.age > DROPOUT_HOLD_T or held['dRel'] <= 0.0:
      self.radar_lead = None
      return lead
    if present and lead['dRel'] <= held['dRel'] + DROPOUT_JUMP_M:
      return lead  # the vision lead agrees with (or is closer than) the radar lead
    return held
