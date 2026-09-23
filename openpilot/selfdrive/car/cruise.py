import math
import numpy as np

from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.cruise_ext import VCruiseHelperSP


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 128
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
# Toyota software set speed. A physical short press is 450-520 ms on the bus (findings9); the PCM itself treats a
# longer hold as a long press and steps its own set speed by 5 per repeat. Err on the long side: if the PCM sees a
# long press and we see a short one both step by 5, the other way round we merely resync to the PCM's +1.
TOYOTA_VIRTUAL_CRUISE_LONG_PRESS = 100
# Toyota software set speed: the PCM will not hold the car more than roughly 8 kph above its OWN set speed even while
# openpilot commands positive acceleration. It cuts throttle somewhere above +11 and only resumes below about +7.5,
# which turns a larger gap into a speed oscillation (option1b_findings14 section 8). The driver's target is therefore
# capped at the PCM set speed plus this headroom, so the HUD never shows a number the car cannot reach.
# 7 km/h keeps the target-to-PCM gap below the ~+11 throttle-cut point (so no oscillation) while giving the + button
# more room before it hits the cap; the PCM holds the car up to about +7-8 (findings14 §8). Raised from 5 after the
# Corolla Cross reported + presses doing nothing once the target sat at the ceiling (rlog 2026-09-20).
TOYOTA_PCM_SET_SPEED_HEADROOM_KPH = 7.
# Toyota's speedometer/cluster reads a fixed ~1.4% high vs the canonical CAN speed (Corolla Cross rlog 2026-09-24:
# vEgoCluster/vEgo ~1.013-1.017 flat with speed, so the gap is +0.5 km/h at 40 but +1.4 at 100 - a fixed +1 only
# matches 60-90). Show the set speed on that same ratio so the displayed set number equals the speedometer at every
# speed: display = canonical * ratio, and the car drives canonical = display / ratio (exact inverse, no rounding gap).
TOYOTA_SPEEDO_RATIO = 1.014
# A long press is how the driver moves the PCM's own number (this car steps it by 1 every ~0.25 s while held). With
# openpilot owning the set speed, the driver uses a long press to park the PCM ceiling high (e.g. 120) once per drive
# and openpilot's own target is left untouched by it; short presses then move openpilot's target by the custom
# increment underneath that ceiling (findings14 §15).
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper(VCruiseHelperSP):
  def __init__(self, CP, CP_SP):
    VCruiseHelperSP.__init__(self, CP, CP_SP)
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_planner_kph = V_CRUISE_UNSET  # what the longitudinal planner tracks, see apply_toyota_pcm_set_speed_constraints
    # Toyota software set speed: remember how cruise was (re)engaged so a RES resume keeps openpilot's own target
    self._toyota_frame = 0
    self._toyota_last_press: tuple[int | None, int] = (None, -10**9)
    self._toyota_engaged_prev = False
    self._toyota_engaged_by_set = True
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  @property
  def software_pcm_cruise_speed(self) -> bool:
    return self.CP.brand == "toyota" and self.CP.pcmCruise and self.CP.openpilotLongitudinalControl and not self.CP_SP.pcmCruiseSpeed

  @property
  def cruise_long_press_frames(self) -> int:
    return TOYOTA_VIRTUAL_CRUISE_LONG_PRESS if self.software_pcm_cruise_speed else CRUISE_LONG_PRESS

  @property
  def software_pcm_cruise_initialized(self) -> bool:
    return 0 < self.v_cruise_kph < V_CRUISE_UNSET and 0 < self.v_cruise_cluster_kph < V_CRUISE_UNSET

  def _apply_software_pcm_cruise_delta(self, delta_kph: float, is_metric: bool) -> None:
    """Apply a delta in DISPLAY (speedometer) space and keep the canonical target = display / speedo ratio, so the
    set number the driver sees and snaps on lands on the speedometer's scale."""
    new_kph = (self.v_cruise_cluster_kph + delta_kph) / TOYOTA_SPEEDO_RATIO
    self.v_cruise_kph = round(float(np.clip(new_kph, self.v_cruise_min, V_CRUISE_MAX)), 1)
    self.v_cruise_cluster_kph = round(self.v_cruise_kph * TOYOTA_SPEEDO_RATIO, 1)

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    self.get_minimum_set_speed(is_metric)

    _enabled = self.update_enabled_state(CS, enabled)
    hold_own_target = self._toyota_hold_own_target(CS, _enabled)

    if CS.cruiseState.available:
      software_pcm_enabled = not self.CP_SP.pcmCruiseSpeed and _enabled
      if self.software_pcm_cruise_speed:
        software_pcm_enabled = software_pcm_enabled and self.software_pcm_cruise_initialized

      if hold_own_target:
        # Toyota software set speed, cruise cancelled or resumed with RES: keep openpilot's own target instead of
        # mirroring the PCM's number, which is the parked ceiling (e.g. 120) and must never become the target
        pass
      elif not self.CP.pcmCruise or software_pcm_enabled:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, _enabled, is_metric)
        v_cruise_kph_before_sla = self.v_cruise_kph
        self.update_speed_limit_assist_v_cruise_non_pcm()
        if self.software_pcm_cruise_speed:
          sla_delta_kph = self.v_cruise_kph - v_cruise_kph_before_sla
          self.v_cruise_kph = v_cruise_kph_before_sla
          self._apply_software_pcm_cruise_delta(sla_delta_kph, is_metric)
        else:
          self.v_cruise_cluster_kph = self.v_cruise_kph
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

    if not self.CP.pcmCruise or not self.CP_SP.pcmCruiseSpeed:
      self.update_button_timers(CS, enabled)

    self.apply_toyota_pcm_set_speed_constraints(CS)

  def _toyota_hold_own_target(self, CS, _enabled: bool) -> bool:
    """Toyota software set speed only. While cruise is cancelled (brake, CANCEL) and through a RES resume, openpilot's
    own target is kept. Only a fresh SET (or an engagement we did not see a RES press for) mirrors the PCM, whose
    number then equals the current speed. Never holds before the target is initialized."""
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
    self._toyota_engaged_prev = engaged

    if _enabled:
      return False  # the software set speed branch owns the target
    if not engaged:
      return True   # cancelled: hold
    return not self._toyota_engaged_by_set  # engagement transition frames: hold after RES, mirror after SET

  def apply_toyota_pcm_set_speed_constraints(self, CS) -> None:
    """Toyota software set speed only. The PCM will not hold the car more than ~8 kph above its own set speed
    (findings14 §8), so the driver's target is capped at the PCM number + headroom. The driver raises that ceiling
    with a long press (the PCM steps by 1 every ~0.25 s while held); openpilot's target does not move on a long press.
    The planner target equals the driver's target after the cap. Other brands and PCM mirroring are untouched."""
    self.v_cruise_planner_kph = self.v_cruise_kph

    if not (self.software_pcm_cruise_speed and self.software_pcm_cruise_initialized and CS.cruiseState.speed > 0):
      return

    pcm_kph = round(CS.cruiseState.speed * CV.MS_TO_KPH, 1)

    self.v_cruise_kph = min(self.v_cruise_kph, round(pcm_kph + TOYOTA_PCM_SET_SPEED_HEADROOM_KPH, 1))
    # show the set speed on the speedometer's scale so set number == speedometer reading at every speed
    self.v_cruise_cluster_kph = round(self.v_cruise_kph * TOYOTA_SPEEDO_RATIO, 1)
    self.v_cruise_planner_kph = self.v_cruise_kph

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    # Toyota software set speed: accumulate EVERY short press (rapid taps included) and apply the total, so N quick
    # +/- taps reliably move the target by N x increment even if they arrive faster than one press per control cycle.
    if self.software_pcm_cruise_speed:
      self._update_software_pcm_short_press(CS, is_metric)
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    # Toyota software set speed: a long press only moves the PCM's own number (the ceiling); openpilot's target is
    # untouched by it. Short presses below move openpilot's target by the custom increment.
    if self.software_pcm_cruise_speed and any(timer >= self.cruise_long_press_frames for timer in self.button_timers.values()):
      return

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > self.cruise_long_press_frames:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % self.cruise_long_press_frames == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    # Speed Limit Assist for Non PCM long cars.
    # True: Disallow set speed changes when user confirmed the target set speed during preActive state
    # False: Allow set speed changes as SLA is not requesting user confirmation
    if self.update_speed_limit_assist_pre_active_confirmed(button_type):
      return

    long_press, v_cruise_delta = VCruiseHelperSP.update_v_cruise_delta(self, long_press, v_cruise_delta)
    # Toyota's canonical PCM set speed and displayed cluster set speed can differ. In
    # software-owned PCM mode, round the value the driver sees and apply the same delta
    # to both targets so the planner/cluster calibration offset remains intact.
    v_cruise_reference = self.v_cruise_cluster_kph if self.software_pcm_cruise_speed else self.v_cruise_kph
    if long_press and v_cruise_reference % v_cruise_delta != 0:  # partial interval
      v_cruise_reference_new = CRUISE_NEAREST_FUNC[button_type](v_cruise_reference / v_cruise_delta) * v_cruise_delta
    else:
      v_cruise_reference_new = v_cruise_reference + v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    if self.software_pcm_cruise_speed:
      delta_kph = v_cruise_reference_new - v_cruise_reference

      # If SET is pressed while overriding, do not lower the target below the current speed.
      if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
        delta_kph = max(delta_kph, CS.vEgo * CV.MS_TO_KPH - self.v_cruise_kph)

      self._apply_software_pcm_cruise_delta(delta_kph, is_metric)
      return

    self.v_cruise_kph += v_cruise_reference_new - v_cruise_reference

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), self.v_cruise_min, V_CRUISE_MAX)

  def _update_software_pcm_short_press(self, CS, is_metric):
    """Software set speed (Toyota): apply EVERY short-press release this cycle, in order, so rapid +/- taps
    accumulate. Each press uses the same snap-or-add rule as a single press (a custom increment of 5/10 snaps the
    displayed number to the nearest multiple in the press direction, otherwise it steps by the increment), so the
    set speed stays on round numbers. A long press (held beyond the threshold) only moves the PCM ceiling and is not
    counted. It need not keep up with the taps in real time; N taps land N steps."""
    base = 1. if is_metric else IMPERIAL_INCREMENT
    for b in CS.buttonEvents:
      if b.type.raw not in self.button_timers or b.pressed:
        continue
      hold = self.button_timers[b.type.raw]
      if hold <= 0 or hold > self.cruise_long_press_frames:
        continue  # not a short press (spurious, or a long press = ceiling move)
      st = self.button_change_states[b.type.raw]
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

  def update_button_timers(self, CS, enabled):
    if self.software_pcm_cruise_speed and (not enabled or not CS.cruiseState.available or not self.software_pcm_cruise_initialized):
      for k in self.button_timers:
        self.button_timers[k] = 0
        self.button_change_states[k] = {"standstill": False, "enabled": False}
      return

    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool, dynamic_experimental_control: bool) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial_experimental_mode = experimental_mode and not dynamic_experimental_control
    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if initial_experimental_mode else V_CRUISE_INITIAL

    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph
