"""Toyota (Corolla Cross TSS2) software-owned cruise set speed: short press +/-5 on openpilot's own target, long press
only moves the PCM ceiling, target capped at PCM + headroom, CANCEL/RES keep the target, dial calibration on the HUD.
Host runnable: only needs Params for the CustomAcc* increments (tncr18 7bbad13422..f89b54e8e0, ported to tncr19)."""
import unittest

from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import (VCruiseHelper, V_CRUISE_UNSET, TOYOTA_VIRTUAL_CRUISE_LONG_PRESS,
                                            TOYOTA_PCM_SET_SPEED_HEADROOM_KPH, toyota_dial_from_canonical,
                                            toyota_canonical_from_dial)

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type

PCM_KPH = 80.      # the PCM's own set speed (0xB4 units) after a SET at ~80
PCM_DIAL_KPH = 86. # what the cluster shows for it (UI_SET_SPEED)


class TestToyotaSoftwareCruise(unittest.TestCase):
  def setUp(self):
    params = Params()
    params.put_bool("CustomAccIncrementsEnabled", True)
    params.put("CustomAccShortPressIncrement", 5)
    params.put("CustomAccLongPressIncrement", 5)

    CP = car.CarParams.new_message(brand="toyota", pcmCruise=True, openpilotLongitudinalControl=True)
    CP_SP = structs.CarParamsSP()
    CP_SP.pcmCruiseSpeed = False
    self.helper = VCruiseHelper(CP, CP_SP)
    self.assertTrue(self.helper.software_pcm_cruise_speed)

    self.pcm_kph = PCM_KPH
    self.pcm_dial_kph = PCM_DIAL_KPH
    self.engaged = True
    self.v_ego_kph = 80.

  def cs(self, events=()):
    CS = car.CarState.new_message()
    CS.cruiseState.available = True
    CS.cruiseState.enabled = self.engaged
    CS.cruiseState.speed = self.pcm_kph * CV.KPH_TO_MS
    CS.cruiseState.speedCluster = self.pcm_dial_kph * CV.KPH_TO_MS
    CS.vEgo = self.v_ego_kph * CV.KPH_TO_MS
    CS.buttonEvents = [ButtonEvent.new_message(type=t, pressed=p) for t, p in events]
    return CS

  def step(self, enabled=True, events=(), n=1):
    for _ in range(n):
      self.helper.update_v_cruise(self.cs(events), enabled, is_metric=True)
      events = ()

  def engage_with_set(self):
    """SET at 80: openpilot sees the PCM number first (controls not yet enabled), then the software branch takes over."""
    self.engaged = True
    self.step(enabled=False, events=[(ButtonType.decelCruise, True)])
    self.step(enabled=False, events=[(ButtonType.decelCruise, False)])
    self.step(enabled=True, n=3)

  def tap(self, button, hold_frames=5):
    self.step(events=[(button, True)])
    self.step(n=hold_frames - 1)
    self.step(events=[(button, False)])

  # --- dial calibration -------------------------------------------------------------------------------------------
  def test_dial_calibration_round_trips(self):
    for dial in (30., 55., 80., 100., 120.):
      self.assertAlmostEqual(toyota_dial_from_canonical(toyota_canonical_from_dial(dial)), dial, places=6)
    # PCM pairs read off the Corolla Cross (rlog 2026-09-25): SET_SPEED 80 (0xB4 units, ~canonical 78.4) shows as dial 86,
    # 112 -> 120; the HUD number IS the dial number, so dial 86 must drive canonical ~78.2
    self.assertAlmostEqual(toyota_canonical_from_dial(86.), 78.2, delta=0.3)
    self.assertAlmostEqual(toyota_dial_from_canonical(112. / 1.021), 120., delta=0.6)

  # --- SET mirrors the PCM once, on the dial's calibration ---------------------------------------------------------
  def test_set_mirrors_pcm_on_dial_calibration(self):
    self.engage_with_set()
    self.assertEqual(self.helper.v_cruise_cluster_kph, PCM_DIAL_KPH)
    self.assertAlmostEqual(self.helper.v_cruise_kph, toyota_canonical_from_dial(PCM_DIAL_KPH), delta=0.1)
    self.assertLess(self.helper.v_cruise_kph, PCM_KPH)  # NOT the raw PCM number (would land ~2 km/h high on the dial)
    self.assertEqual(self.helper.v_cruise_planner_kph, self.helper.v_cruise_kph)

  # --- short press +/-5 snaps the displayed number onto the 5 km/h grid --------------------------------------------
  def test_short_press_steps_the_display_by_the_increment(self):
    self.engage_with_set()
    self.tap(ButtonType.accelCruise)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 90.)  # 86 snaps up to 90
    self.assertAlmostEqual(self.helper.v_cruise_kph, toyota_canonical_from_dial(90.), delta=0.1)
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 85.)
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 80.)
    self.assertEqual(self.helper.v_cruise_planner_kph, self.helper.v_cruise_kph)

  def test_rapid_taps_accumulate(self):
    self.engage_with_set()
    for _ in range(3):
      self.tap(ButtonType.decelCruise, hold_frames=3)  # 86 -> 85 -> 80 -> 75, three taps in 90 ms
    self.assertEqual(self.helper.v_cruise_cluster_kph, 75.)
    self.tap(ButtonType.accelCruise, hold_frames=2)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 80.)

  # --- long press moves only the PCM ceiling ----------------------------------------------------------------------
  def test_long_press_leaves_the_target_alone(self):
    self.engage_with_set()
    target, cluster = self.helper.v_cruise_kph, self.helper.v_cruise_cluster_kph
    self.step(events=[(ButtonType.accelCruise, True)])
    for i in range(TOYOTA_VIRTUAL_CRUISE_LONG_PRESS + 40):
      if i % 25 == 0:  # the PCM steps its own number by 1 every ~0.25 s while held
        self.pcm_kph += 1
        self.pcm_dial_kph += 1
      self.step()
    self.step(events=[(ButtonType.accelCruise, False)])
    self.assertEqual(self.helper.v_cruise_kph, target)
    self.assertEqual(self.helper.v_cruise_cluster_kph, cluster)
    self.assertGreater(self.pcm_kph, PCM_KPH)

  # --- the planner target is capped at PCM + headroom -------------------------------------------------------------
  def test_target_capped_at_pcm_plus_headroom(self):
    self.engage_with_set()
    for _ in range(6):
      self.tap(ButtonType.accelCruise)
    cap = round(PCM_KPH + TOYOTA_PCM_SET_SPEED_HEADROOM_KPH, 1)
    self.assertEqual(self.helper.v_cruise_kph, cap)
    self.assertEqual(self.helper.v_cruise_planner_kph, cap)
    self.assertEqual(self.helper.v_cruise_cluster_kph, toyota_dial_from_canonical(cap))
    # parking the ceiling higher releases the cap for the next + press
    self.pcm_kph, self.pcm_dial_kph = 120., 128.
    self.tap(ButtonType.accelCruise)
    self.assertGreater(self.helper.v_cruise_kph, cap)
    self.assertEqual(self.helper.v_cruise_cluster_kph % 5, 0)

  # --- CANCEL and a RES resume keep openpilot's target; SET mirrors again ------------------------------------------
  def test_cancel_then_res_keeps_the_target(self):
    self.engage_with_set()
    self.tap(ButtonType.decelCruise)
    self.tap(ButtonType.decelCruise)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 80.)
    target = self.helper.v_cruise_kph
    # driver cancels; the PCM keeps its (higher) parked number
    self.engaged = False
    self.pcm_kph, self.pcm_dial_kph = 120., 128.
    self.step(enabled=False, n=20)
    self.assertEqual(self.helper.v_cruise_kph, target)
    # RES: the PCM re-engages at its parked number, openpilot keeps its own
    self.step(enabled=False, events=[(ButtonType.accelCruise, True)])
    self.engaged = True
    self.step(enabled=False, events=[(ButtonType.accelCruise, False)])
    self.step(enabled=False, n=5)
    self.step(enabled=True, n=5)
    self.assertEqual(self.helper.v_cruise_kph, target)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 80.)

  def test_cancel_then_set_mirrors_the_pcm(self):
    self.engage_with_set()
    self.tap(ButtonType.decelCruise)
    self.engaged = False
    self.step(enabled=False, n=20)
    # SET at the current speed: PCM takes 70 (dial 76), openpilot mirrors it
    self.pcm_kph, self.pcm_dial_kph = 70., 76.
    self.step(enabled=False, events=[(ButtonType.decelCruise, True)])
    self.engaged = True
    self.step(enabled=False, events=[(ButtonType.decelCruise, False)])
    self.step(enabled=False, n=5)
    self.assertEqual(self.helper.v_cruise_cluster_kph, 76.)
    self.assertAlmostEqual(self.helper.v_cruise_kph, toyota_canonical_from_dial(76.), delta=0.1)

  def test_unset_when_cruise_unavailable(self):
    self.engage_with_set()
    CS = self.cs()
    CS.cruiseState.available = False
    self.helper.update_v_cruise(CS, True, is_metric=True)
    self.assertEqual(self.helper.v_cruise_kph, V_CRUISE_UNSET)
    self.assertEqual(self.helper.v_cruise_planner_kph, V_CRUISE_UNSET)


if __name__ == "__main__":
  unittest.main()
