"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import ACTIVE_STATES as SLA_ACTIVE_STATES
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import compare_cluster_target

ButtonType = car.CarState.ButtonEvent.Type
SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

CRUISE_BUTTON_TIMER = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0,
                       ButtonType.setCruise: 0, ButtonType.resumeCruise: 0,
                       ButtonType.cancel: 0, ButtonType.mainCruise: 0}

V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255

# Toyota software set speed: the PCM will not hold the car more than roughly 8 kph above its OWN set speed even while
# openpilot commands positive acceleration. It cuts throttle somewhere above +11 and only resumes below about +7.5,
# which turns a larger gap into a speed oscillation (option1b_findings14 section 8). The driver's target is therefore
# capped at the PCM set speed plus this headroom, so the HUD never shows a number the car cannot reach.
# 7 km/h keeps the target-to-PCM gap below the ~+11 throttle-cut point (so no oscillation) while giving the + button
# more room before it hits the cap; the PCM holds the car up to about +7-8 (findings14 §8). Raised from 5 after the
# Corolla Cross reported + presses doing nothing once the target sat at the ceiling (rlog 2026-09-20, tncr18 a2e52997a8).
TOYOTA_PCM_SET_SPEED_HEADROOM_KPH = 7.
# Toyota's speedometer is NOT the canonical CAN speed. The PCM keeps its set speed in two units: SET_SPEED (internal,
# in the same units as the 0xB4 speed signal = ~true speed) and UI_SET_SPEED (what the cluster shows). Pairs read off
# the Corolla Cross while the ceiling was being long-pressed up (rlog 2026-09-25, route 47): 26->30, 50->55, 74->80,
# 93->100, 112->120, 120->128. Least squares: UI = 1.0457*internal + 2.73 (max residual 0.5, the internal value is
# an integer). The dial follows the same calibration, so a target the driver reads on the HUD must be converted on
# that line, not by the 1.021 that 0xB4/vEgo alone gives. Combined with 0xB4/vEgo = 1.021:
#   dial = TOYOTA_DIAL_SLOPE * canonical + TOYOTA_DIAL_OFFSET_KPH
# Road check the same day: a displayed 80 drove canonical 78.4 while the dial read 86; a displayed 100 drove 97.9
# and the dial read 107. The line predicts 86.4 and 107.2. (The earlier 1.014/1.021 ratios only used vEgoCluster or
# 0xB4, neither of which is the dial, hence the 7% miss.) The car drives canonical = (display - offset) / slope
# (tncr18 f89b54e8e0).
TOYOTA_DIAL_SLOPE = 1.0457 * 1.021
TOYOTA_DIAL_OFFSET_KPH = 2.53  # 2.73 measured; -0.2 on 2026-09-26 so the dial sits ~0.2 above the set number instead of
                                #  fractionally below it (the driver saw the dial round down to N-1 at times)
# A long press is how the driver moves the PCM's own number (this car steps it by 1 every ~0.25 s while held). With
# openpilot owning the set speed, the driver uses a long press to park the PCM ceiling high (e.g. 120) once per drive
# and openpilot's own target is left untouched by it; short presses then move openpilot's target by the custom
# increment underneath that ceiling (findings14 §15, tncr18 47f4349310).


def toyota_dial_from_canonical(kph: float) -> float:
  if kph <= 0.0:
    return 0.0  # the stop target (set speed 0) shows 0
  dial = TOYOTA_DIAL_SLOPE * kph + TOYOTA_DIAL_OFFSET_KPH
  # the canonical target is kept to 0.1, which can put the dial number 0.1 off the integer the driver dialed; snap
  # back onto it so the HUD shows the round number and the +/- snap grid keeps working
  nearest = round(dial)
  return float(nearest) if abs(dial - nearest) < 0.15 else round(dial, 1)


def toyota_canonical_from_dial(kph: float) -> float:
  return max(0.0, (kph - TOYOTA_DIAL_OFFSET_KPH) / TOYOTA_DIAL_SLOPE)


def update_manual_button_timers(CS: car.CarState, button_timers: dict[car.CarState.ButtonEvent.Type, int]) -> None:
  # increment timer for buttons still pressed
  for k in button_timers:
    if button_timers[k] > 0:
      button_timers[k] += 1

  for b in CS.buttonEvents:
    if b.type.raw in button_timers:
      # Start/end timer and store current state on change of button pressed
      button_timers[b.type.raw] = 1 if b.pressed else 0


class VCruiseHelperSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP = CP
    self.CP_SP = CP_SP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.params = Params()
    self.v_cruise_min = 0
    self.enabled_prev = False

    self.custom_acc_enabled = self.params.get_bool("CustomAccIncrementsEnabled")
    self.short_increment = self.params.get("CustomAccShortPressIncrement", return_default=True)
    self.long_increment = self.params.get("CustomAccLongPressIncrement", return_default=True)

    self.enable_button_timers = CRUISE_BUTTON_TIMER

    # Speed Limit Assist
    self.sla_state = SpeedLimitAssistState.disabled
    self.prev_sla_state = SpeedLimitAssistState.disabled
    self.has_speed_limit = False
    self.speed_limit_final_last = 0.
    self.speed_limit_final_last_kph = 0.
    self.prev_speed_limit_final_last_kph = 0.
    self.req_plus = False
    self.req_minus = False

    self.v_cruise_planner_kph = V_CRUISE_UNSET  # what the longitudinal planner tracks, see pcm_ceiling_sp
    # Toyota software set speed: remember how cruise was (re)engaged so a RES resume keeps openpilot's own target
    # (tncr18 30449669e7)
    self._toyota_frame = 0
    self._toyota_last_press: tuple[int | None, int] = (None, -10**9)
    self._toyota_engaged_prev = False
    self._toyota_engaged_by_set = True

  def read_custom_set_speed_params(self) -> None:
    self.custom_acc_enabled = self.params.get_bool("CustomAccIncrementsEnabled")
    self.short_increment = self.params.get("CustomAccShortPressIncrement", return_default=True)
    self.long_increment = self.params.get("CustomAccLongPressIncrement", return_default=True)

  def update_v_cruise_delta(self, long_press: bool, v_cruise_delta: float) -> tuple[bool, float]:
    if not self.custom_acc_enabled:
      v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
      return long_press, v_cruise_delta

    # Apply user-specified multipliers to the base increment
    short_increment = np.clip(self.short_increment, 1, 10)
    long_increment = np.clip(self.long_increment, 1, 10)

    actual_increment = long_increment if long_press else short_increment
    round_to_nearest = actual_increment in (5, 10)
    v_cruise_delta = v_cruise_delta * actual_increment

    return round_to_nearest, v_cruise_delta

  def get_minimum_set_speed(self, is_metric: bool) -> None:
    if self.software_pcm_cruise_speed:
      # Toyota software set speed (tncr, owner 2026-10-27): the -5 presses go on below 30 km/h down to 0, a stop
      # target - with no lead the car slows on the eco schedule and stops (accel_controller.py); RES resumes to the
      # PCM ceiling (resume_to_ceiling_sp)
      self.v_cruise_min = 0
      return
    if self.CP_SP.pcmCruiseSpeed:
      self.v_cruise_min = V_CRUISE_MIN
      return

    self.v_cruise_min = get_minimum_set_speed(is_metric)

  def update_enabled_state(self, CS: car.CarState, enabled: bool) -> bool:
    # special enabled state for non pcmCruiseSpeed, unchanged for non pcmCruise
    if not self.CP_SP.pcmCruiseSpeed:
      update_manual_button_timers(CS, self.enable_button_timers)
      button_pressed = any(self.enable_button_timers[k] > 0 for k in self.enable_button_timers)

      if enabled and not self.enabled_prev:
        self.enabled_prev = not button_pressed
        enabled = False
      elif not enabled:
        self.enabled_prev = enabled

      return enabled and self.enabled_prev

    return enabled

  def update_speed_limit_assist(self, is_metric, LP_SP: custom.LongitudinalPlanSP) -> None:
    resolver = LP_SP.speedLimit.resolver
    self.has_speed_limit = resolver.speedLimitValid or resolver.speedLimitLastValid
    self.speed_limit_final_last = LP_SP.speedLimit.resolver.speedLimitFinalLast
    self.speed_limit_final_last_kph = self.speed_limit_final_last * CV.MS_TO_KPH
    self.sla_state = LP_SP.speedLimit.assist.state
    self.req_plus, self.req_minus = compare_cluster_target(self.v_cruise_cluster_kph * CV.KPH_TO_MS,
                                                           self.speed_limit_final_last, is_metric)

  @property
  def update_speed_limit_final_last_changed(self) -> bool:
    return self.has_speed_limit and bool(self.speed_limit_final_last_kph != self.prev_speed_limit_final_last_kph)

  def update_speed_limit_assist_pre_active_confirmed(self, button_type: car.CarState.ButtonEvent.Type) -> bool:
    if self.sla_state == SpeedLimitAssistState.preActive or self.prev_sla_state == SpeedLimitAssistState.preActive:
      if button_type == ButtonType.decelCruise and self.req_minus:
        return True
      if button_type == ButtonType.accelCruise and self.req_plus:
        return True

    return False

  def update_speed_limit_assist_v_cruise_non_pcm(self) -> None:
    if self.sla_state in SLA_ACTIVE_STATES and (self.prev_sla_state not in SLA_ACTIVE_STATES or
                                                self.update_speed_limit_final_last_changed):
      self.v_cruise_kph = np.clip(round(self.speed_limit_final_last_kph, 1), self.v_cruise_min, V_CRUISE_MAX)

    self.prev_sla_state = self.sla_state
    self.prev_speed_limit_final_last_kph = self.speed_limit_final_last_kph

  def apply_dial_cruise_delta_sp(self, delta_kph: float) -> bool:
    """Toyota software set speed: apply a delta in DISPLAY (speedometer) space and keep the canonical target on the
    dial's calibration line, so the set number the driver sees and snaps on lands on the speedometer's scale (tncr18
    8a4cb53d8a + f89b54e8e0). The display number is re-derived from the bounded canonical target, so both stay inside
    [v_cruise_min, V_CRUISE_MAX]. Stock calls this only for the software set speed; always handled."""
    new_kph = toyota_canonical_from_dial(self.v_cruise_cluster_kph + delta_kph)
    self.v_cruise_kph = round(float(np.clip(new_kph, self.v_cruise_min, V_CRUISE_MAX)), 1)
    self.v_cruise_cluster_kph = toyota_dial_from_canonical(self.v_cruise_kph)
    return True

  def mirror_pcm_target_sp(self, CS, v_cruise_kph: float) -> float:
    """A fresh SET mirrors the PCM: keep the cluster's number and drive the canonical speed that puts the dial on it
    (the PCM's own SET_SPEED is in 0xB4 units and would land ~2 km/h high on the dial, tncr18 f89b54e8e0)."""
    if self.software_pcm_cruise_speed and CS.cruiseState.speedCluster > 0:
      return round(toyota_canonical_from_dial(self.v_cruise_cluster_kph), 1)
    return v_cruise_kph

  def hold_own_target_sp(self, CS, _enabled: bool) -> bool:
    """Toyota software set speed only. While cruise is cancelled (brake, CANCEL) and through a RES resume, openpilot's
    own target is kept. Only a fresh SET (or an engagement we did not see a RES press for) mirrors the PCM, whose
    number then equals the current speed. Never holds before the target is initialized (tncr18 30449669e7)."""
    if not (self.software_pcm_cruise_speed and self.software_pcm_cruise_initialized):
      return False

    self._toyota_frame += 1
    for b in CS.buttonEvents:
      if b.pressed and b.type in (ButtonType.accelCruise, ButtonType.decelCruise):
        self._toyota_last_press = (b.type.raw, self._toyota_frame)

    engaged = CS.cruiseState.enabled
    if engaged and not self._toyota_engaged_prev:
      press_type, press_frame = self._toyota_last_press
      resumed_with_res = press_type == ButtonType.accelCruise and self._toyota_frame - press_frame < 150
      self._toyota_engaged_by_set = not resumed_with_res
      if resumed_with_res and self.v_cruise_kph == 0:
        self.resume_to_ceiling_sp(CS)  # cancelled while stopping to 0: RES goes back up to the ceiling
    self._toyota_engaged_prev = engaged

    if _enabled:
      return False  # the software set speed branch owns the target
    if not engaged:
      return True   # cancelled: hold
    return not self._toyota_engaged_by_set  # engagement transition frames: hold after RES, mirror after SET

  def resume_to_ceiling_sp(self, CS) -> None:
    """From the stop target (0): the target goes back to the PCM ceiling, the number on the dash (owner 2026-10-27)."""
    ceiling_dial = CS.cruiseState.speedCluster * CV.MS_TO_KPH
    if ceiling_dial <= 0.0:
      return
    self.v_cruise_kph = round(float(np.clip(toyota_canonical_from_dial(ceiling_dial), self.v_cruise_min, V_CRUISE_MAX)), 1)
    self.v_cruise_cluster_kph = toyota_dial_from_canonical(self.v_cruise_kph)

  def pcm_ceiling_sp(self, CS) -> None:
    """Toyota software set speed only. The PCM will not hold the car more than ~8 kph above its own set speed
    (findings14 §8), so the driver's target is capped at the PCM number + headroom. The driver raises that ceiling
    with a long press (the PCM steps by 1 every ~0.25 s while held); openpilot's target does not move on a long press.
    The planner target equals the driver's target after the cap. Other brands and PCM mirroring are untouched
    (tncr18 ce25c42170 + cefe9d7fe4)."""
    self.v_cruise_planner_kph = self.v_cruise_kph

    if not (self.software_pcm_cruise_speed and self.software_pcm_cruise_initialized and CS.cruiseState.speed > 0):
      return

    pcm_kph = round(CS.cruiseState.speed * CV.MS_TO_KPH, 1)

    self.v_cruise_kph = min(self.v_cruise_kph, round(pcm_kph + TOYOTA_PCM_SET_SPEED_HEADROOM_KPH, 1))
    # show the set speed on the speedometer's scale so set number == speedometer reading at every speed
    self.v_cruise_cluster_kph = toyota_dial_from_canonical(self.v_cruise_kph)
    self.v_cruise_planner_kph = self.v_cruise_kph

  def software_pcm_short_press_sp(self, CS, is_metric) -> bool:
    """Software set speed (Toyota): apply EVERY short-press release this cycle, in order, so rapid +/- taps
    accumulate. Each press uses the same snap-or-add rule as a single press (a custom increment of 5/10 snaps the
    displayed number to the nearest multiple in the press direction, otherwise it steps by the increment), so the
    set speed stays on round numbers. A long press (held beyond the threshold) only moves the PCM ceiling and is not
    counted. It need not keep up with the taps in real time; N taps land N steps (tncr18 2d2dd4a0ee + 2b4c19d9e8).
    Returns True when it handled the presses (Toyota software set speed), False to let stock handle them."""
    if not self.software_pcm_cruise_speed:
      return False
    # stock cruise.py imports this module, so its button tables are imported here, at call time
    from openpilot.selfdrive.car.cruise import CRUISE_INTERVAL_SIGN, CRUISE_NEAREST_FUNC, IMPERIAL_INCREMENT
    base = 1. if is_metric else IMPERIAL_INCREMENT
    for b in CS.buttonEvents:
      if b.type.raw not in self.button_timers or b.pressed:
        continue
      hold = self.button_timers[b.type.raw]
      if hold <= 0 or hold > self.cruise_long_press_frames:
        continue  # not a short press (spurious, or a long press = ceiling move)
      st = self.button_change_states[b.type.raw]
      if b.type.raw == ButtonType.accelCruise and self.v_cruise_kph == 0 and st["enabled"]:
        self.resume_to_ceiling_sp(CS)  # set speed 0 (stopping or stopped): RES / + goes back up to the ceiling
        continue
      if b.type.raw == ButtonType.accelCruise and (st["standstill"] or CS.cruiseState.standstill):
        continue
      if not st["enabled"]:
        continue
      if self.update_speed_limit_assist_pre_active_confirmed(b.type.raw):
        continue

      round_to_nearest, delta = VCruiseHelperSP.update_v_cruise_delta(self, False, base)
      ref = self.v_cruise_cluster_kph
      if round_to_nearest and ref % delta != 0:  # snap the displayed number onto the increment grid
        ref_new = CRUISE_NEAREST_FUNC[b.type.raw](ref / delta) * delta
      else:
        ref_new = ref + delta * CRUISE_INTERVAL_SIGN[b.type.raw]
      delta_kph = ref_new - ref
      # if SET is tapped while overriding, do not lower the target below the current speed
      if CS.gasPressed and delta_kph < 0.:
        delta_kph = max(delta_kph, CS.vEgo * CV.MS_TO_KPH - self.v_cruise_kph)
      self._apply_software_pcm_cruise_delta(delta_kph, is_metric)
    return True
