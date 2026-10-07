"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.toyota.values import ToyotaFlags
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP
from opendbc.sunnypilot.car.toyota.wheel_pulse import WheelPulseCreep
from opendbc.sunnypilot.car.toyota.values import get_tn_switch

ENGINE_RUNNING_RPM = 300.0

TRAFFIC_SIGNAL_MAP = {
  1: "kph",
  36: "mph",
  65: "No overtake",
  66: "No overtake"
}

ENGINE_RUNNING_RPM = 300.  # hybrid: below this the engine is stopped (Corolla rlogs: 0 when off, >=800 when on)
ZSS_DIFF_THRESHOLD = 4
ZSS_MAX_THRESHOLD = 10


# Fitted 2026-10-02 to the Altis dash at steady cruise (true speed from the logs, dash read by the driver): v 103.00 ->
# dash 110, 92.99 -> 99, 63.96 -> 69 (routes 000000e9/ea), 98.0 -> 104 (09-30). 1.05 v + 1.47 rounds to every one of them
# (margin 0.12 km/h); the old 1.04 v + 1.9 showed 109 / 98 / 68 because the 0.5 km/h display hysteresis kept the value up
# to 0.5 below after accelerating to the set speed (see CLUSTER_HYST_KPH).
CLUSTER_SPEED_GAIN = 1.05           # dash km/h per true km/h (stock openpilot: 1.015)
CLUSTER_SPEED_OFFSET_KPH = 1.47     # km/h added on top
CLUSTER_HYST_KPH = 0.1              # display hysteresis (stock 0.5): steady-cruise vEgo noise is ~+-0.08 km/h
CLUSTER_MIN_KPH = 5.0               # below this the screen shows the true speed
# Set speed on the same scale (owner 2026-10-07 22:50: "set speed 100, the dash and the C3X both show 99 - make them
# agree"). Route 00000110: the PCM turns the dash set speed into its own integer target (UI 100 -> SET_SPEED 93,
# UI 98 -> 91) and openpilot cruised exactly there (vEgo 92.99 / 91.00), which this speedometer shows as 99.1 / 97.0.
# So the cruise target is taken from the speedometer model instead: the true speed at which the dash reads the set
# speed plus SET_SPEED_SCREEN_MARGIN_KPH (100 -> 94.1 km/h, dash 100.3; 98 -> 92.2, dash 98.3). Metric dash only.
SET_SPEED_SCREEN_MARGIN_KPH = 0.3


def set_speed_target(ui_set_kph: float) -> float:
  """true speed (m/s) at which the speedometer reads the dash set speed (+ margin)"""
  return (ui_set_kph + SET_SPEED_SCREEN_MARGIN_KPH - CLUSTER_SPEED_OFFSET_KPH) / CLUSTER_SPEED_GAIN * CV.KPH_TO_MS


def cluster_speed(v_ego: float) -> float:
  """Speed for the on-screen readout, matched to the car's own speedometer (m/s in, m/s out)."""
  v_kph = v_ego * CV.MS_TO_KPH
  if v_kph < CLUSTER_MIN_KPH:
    return v_ego
  return (CLUSTER_SPEED_GAIN * v_kph + CLUSTER_SPEED_OFFSET_KPH) * CV.KPH_TO_MS


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.acc_type = 1
    self.zss_compute = False

    # tnpb2 additions, called from hooks in the stock Toyota CarState (see the *_sp methods at the end)
    from opendbc.car.toyota.carstate import get_host_params
    self.params = get_host_params()
    self.cluster_speed_enabled = self.params is None or get_tn_switch(self.params, "TnToyotaClusterSpeed")
    self.wheel_pulse = (WheelPulseCreep() if self.params is None or
                        get_tn_switch(self.params, "TnToyotaWheelPulse") else None)
    self.wheel_encoder = float('nan')  # SPEED (0xB4) ENCODER: wheel pulse counter, still counts below the ~0.5 km/h speed floor
    self.zss_cruise_active_last = False
    self.zss_angle_offset = 0.
    self.zss_threshold_count = 0

    """Initialize traffic signal variables"""
    self._tsgn1 = None
    self._spdval1 = None
    self._splsgn1 = None
    self._tsgn2 = None
    self._splsgn2 = None
    self._tsgn3 = None
    self._splsgn3 = None
    self._tsgn4 = None
    self._splsgn4 = None

  @staticmethod
  def traffic_signal_description(tsgn):
    """Get description for traffic signal code"""
    desc = TRAFFIC_SIGNAL_MAP.get(int(tsgn))
    return f'{tsgn}: {desc}' if desc is not None else f'{tsgn}'

  def update_traffic_signals(self, cp_cam):
    """Update traffic signals with error handling"""
    try:
      # Add error handling for missing RSA messages
      tsgn1 = cp_cam.vl.get("RSA1", {}).get('TSGN1', 0)
      spdval1 = cp_cam.vl.get("RSA1", {}).get('SPDVAL1', 0)
      splsgn1 = cp_cam.vl.get("RSA1", {}).get('SPLSGN1', 0)
      tsgn2 = cp_cam.vl.get("RSA1", {}).get('TSGN2', 0)
      splsgn2 = cp_cam.vl.get("RSA1", {}).get('SPLSGN2', 0)
      tsgn3 = cp_cam.vl.get("RSA2", {}).get('TSGN3', 0)
      splsgn3 = cp_cam.vl.get("RSA2", {}).get('SPLSGN3', 0)
      tsgn4 = cp_cam.vl.get("RSA2", {}).get('TSGN4', 0)
      splsgn4 = cp_cam.vl.get("RSA2", {}).get('SPLSGN4', 0)
    except (KeyError, AttributeError) as e:
      # Handle case where RSA messages are not available
      carlog.debug(f"RSA messages not available: {e}")
      return

    has_changed = tsgn1 != self._tsgn1 \
                  or spdval1 != self._spdval1 \
                  or splsgn1 != self._splsgn1 \
                  or tsgn2 != self._tsgn2 \
                  or splsgn2 != self._splsgn2 \
                  or tsgn3 != self._tsgn3 \
                  or splsgn3 != self._splsgn3 \
                  or tsgn4 != self._tsgn4 \
                  or splsgn4 != self._splsgn4

    self._tsgn1 = tsgn1
    self._spdval1 = spdval1
    self._splsgn1 = splsgn1
    self._tsgn2 = tsgn2
    self._splsgn2 = splsgn2
    self._tsgn3 = tsgn3
    self._splsgn3 = splsgn3
    self._tsgn4 = tsgn4
    self._splsgn4 = splsgn4

    if not has_changed:
      return

    carlog.debug('---- TRAFFIC SIGNAL UPDATE -----')
    if tsgn1 is not None and tsgn1 != 0:
      carlog.debug(f'TSGN1: {self.traffic_signal_description(tsgn1)}')
    if spdval1 is not None and spdval1 != 0:
      carlog.debug(f'SPDVAL1: {spdval1}')
    if splsgn1 is not None and splsgn1 != 0:
      carlog.debug(f'SPLSGN1: {splsgn1}')
    if tsgn2 is not None and tsgn2 != 0:
      carlog.debug(f'TSGN2: {self.traffic_signal_description(tsgn2)}')
    if splsgn2 is not None and splsgn2 != 0:
      carlog.debug(f'SPLSGN2: {splsgn2}')
    if tsgn3 is not None and tsgn3 != 0:
      carlog.debug(f'TSGN3: {self.traffic_signal_description(tsgn3)}')
    if splsgn3 is not None and splsgn3 != 0:
      carlog.debug(f'SPLSGN3: {splsgn3}')
    if tsgn4 is not None and tsgn4 != 0:
      carlog.debug(f'TSGN4: {self.traffic_signal_description(tsgn4)}')
    if splsgn4 is not None and splsgn4 != 0:
      carlog.debug(f'SPLSGN4: {splsgn4}')
    carlog.debug('------------------------')

  def calculate_speed_limit(self):
    """Calculate speed limit from traffic signals with validation"""
    # Check all traffic sign slots for speed limits, not just tsgn1
    for tsgn, spdval in [(self._tsgn1, self._spdval1),
                         (self._tsgn2, None),
                         (self._tsgn3, None),
                         (self._tsgn4, None)]:

      if tsgn == 1 and spdval is not None and 0 < spdval <= 200:  # Reasonable speed range
        return spdval * CV.KPH_TO_MS

      if tsgn == 36 and spdval is not None and 0 < spdval <= 120:  # Reasonable MPH range
        return spdval * CV.MPH_TO_MS

    return 0

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    if self.CP_SP.flags & ToyotaFlagsSP.SMART_DSU:
      self.acc_type = 1

    if self.CP_SP.enableGasInterceptor:
      gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
      ret.gasPressed = gas > 805

    # ZSS support thanks to zorrobyte, ErichMoraga, and dragonpilot
    if self.CP_SP.flags & ToyotaFlagsSP.ZSS:
      zorro_steer = cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"]
      control_available = ret.cruiseState.available

      # Only compute ZSS offset when control is available
      if control_available and not self.zss_cruise_active_last:
        self.zss_threshold_count = 0
        self.zss_compute = True  # Control was just activated, so allow offset to be recomputed
      self.zss_cruise_active_last = control_available

      # Compute ZSS offset once we have meaningful angles
      if self.zss_compute and abs(ret.steeringAngleDeg) > 1e-3 and abs(zorro_steer) > 1e-3:
        self.zss_compute = False
        self.zss_angle_offset = zorro_steer - ret.steeringAngleDeg

      # Sanity checks
      steering_angle_deg = zorro_steer - self.zss_angle_offset
      if self.zss_threshold_count <= ZSS_MAX_THRESHOLD:
        if abs(ret.steeringAngleDeg - steering_angle_deg) > ZSS_DIFF_THRESHOLD:
          self.zss_threshold_count += 1
        else:
          ret.steeringAngleDeg = steering_angle_deg

    # Update traffic signals and speed limit
    self.update_traffic_signals(cp_cam)
    ret_sp.speedLimit = self.calculate_speed_limit()

    # hybrid engine state from ENGINE_RPM (0x1C4): lets the eco accel profile stay on the battery
    if self.CP.flags & ToyotaFlags.HYBRID and "ENGINE_RPM" in cp.vl:
      ret_sp.engineRpm = float(cp.vl["ENGINE_RPM"]["RPM"])
      ret_sp.engineOff = ret_sp.engineRpm < ENGINE_RUNNING_RPM and not bool(cp.vl["ENGINE_RPM"]["ENGINE_RUNNING"])

  def cluster_hyst_gap_sp(self, stock_gap: float) -> float:
    return CLUSTER_HYST_KPH * CV.KPH_TO_MS if self.cluster_speed_enabled else stock_gap

  # Speedometer model for the on-screen speed. Stock: vEgo * 1.015. Corolla Altis Hybrid 2026-09-30 22:30: at a set
  # speed of 105 the dash read 104 while the screen (1.015 * vEgo) read 100, i.e. vEgo ~98.5 km/h - the dash is about
  # 5.5% above the true speed. Toyota speedometers are close to affine (gain plus a small offset); start from gain 1.04
  # and +1.6 km/h, which reproduces that point, and refine with more dash/screen pairs. Only above CLUSTER_MIN_KPH so
  # crawling and standstill stay exact. Tunables: CLUSTER_SPEED_GAIN, CLUSTER_SPEED_OFFSET_KPH.
  def cluster_speed_sp(self, v_ego: float, stock_value: float) -> float:
    return cluster_speed(v_ego) if self.cluster_speed_enabled else stock_value

  def set_speed_target_sp(self, stock_speed: float, cluster_set_speed: float, is_metric: bool) -> float:
    """cruise target on the speedometer's scale (see SET_SPEED_SCREEN_MARGIN_KPH); stock when off or not metric"""
    ui_kph = cluster_set_speed * CV.MS_TO_KPH
    if not self.cluster_speed_enabled or not is_metric or stock_speed <= 0.0 or ui_kph < 2 * CLUSTER_MIN_KPH:
      return stock_speed
    return set_speed_target(ui_kph)

  def wheel_pulse_creep_sp(self, ret, cp) -> None:
    """below the ~0.5 km/h speed floor the wheel pulse counter still shows a creeping car (wheel_pulse.py)"""
    if self.wheel_pulse is None:
      return
    if "SPEED" in cp.vl:
      self.wheel_encoder = float(cp.vl["SPEED"]["ENCODER"])
    creep_v = self.wheel_pulse.update(self.wheel_encoder, abs(ret.vEgoRaw) < 1e-3)
    if creep_v > 0.0:
      ret.vEgoRaw = creep_v
      ret.vEgo = max(ret.vEgo, creep_v)

  @staticmethod
  def extra_pt_messages_sp(name_to_msg) -> list:
    # wheel pulse counter (0xB4 ENCODER): the wheel speeds read 0 below ~0.5 km/h, the encoder still counts, so a
    # slow creep is not mistaken for a stop (wheel_pulse.py). Alive check skipped.
    from opendbc.car.toyota.carstate import get_host_params
    params = get_host_params()
    if (params is None or get_tn_switch(params, "TnToyotaWheelPulse")) and "SPEED" in name_to_msg:
      return [("SPEED", float('nan'))]
    return []
