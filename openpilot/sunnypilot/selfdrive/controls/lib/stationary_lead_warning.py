"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import messaging, custom
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

# Early "stopped vehicle ahead" warning. On the Corolla Cross the fused lead never appeared beyond ~130 m and the
# vision lead's 99th percentile is ~110 m (rlogs 2026-09-25, > 72 km/h), so at highway speed a stopped car shows up
# with only 4-5 s to go and the stock FCW fires with < 2 s. Warn as soon as a (near-)stationary lead is confirmed
# while closing fast, i.e. essentially at first sight. Vehicles only: debris, cones or barriers are not detected by
# anything openpilot has, so they cannot be warned about.
MIN_V_EGO = 19.4          # m/s (70 km/h)
MAX_LEAD_SPEED = 2.8      # m/s (10 km/h): the lead counts as stopped
MIN_CLOSING_SPEED = 12.0  # m/s: we are actually running up on it
TTC_ON = 5.0              # s: time-to-collision to start warning (> detection range at 90-110 km/h = first sight)
TTC_OFF = 6.5             # s: hysteresis to stop
MIN_DIST = 6.0            # m: ignore the stop-and-go regime
CONFIRM_TIME = 0.3        # s the condition must hold before the first chime (filters one-frame ghosts)


class StationaryLeadWarning:
  def __init__(self):
    self.confirm_frames = 0
    self.active = False
    self.ttc = 0.0

  def update(self, sm: messaging.SubMaster, events_sp: EventsSP) -> None:
    CS = sm['carState']
    lead = sm['radarState'].leadOne

    closing = -lead.vRel
    self.ttc = lead.dRel / closing if closing > 0.1 else 99.0
    candidate = (lead.present and CS.vEgo > MIN_V_EGO and lead.vLead < MAX_LEAD_SPEED and closing > MIN_CLOSING_SPEED
                 and lead.dRel > MIN_DIST and not CS.brakePressed and not CS.gasPressed)

    if candidate and self.ttc < (TTC_OFF if self.active else TTC_ON):
      self.confirm_frames += 1
    else:
      self.confirm_frames = 0
      self.active = False

    if self.confirm_frames * DT_MDL >= CONFIRM_TIME:
      self.active = True

    if self.active:
      events_sp.add(custom.OnroadEventSP.EventName.stationaryLeadWarning)
