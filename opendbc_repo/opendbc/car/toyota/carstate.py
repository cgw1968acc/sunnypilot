import copy
from cereal import custom
from openpilot.common.params import Params
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase
from opendbc.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, \
                                  TSS2_CAR, RADAR_ACC_CAR, EPS_SCALE, UNSUPPORTED_DSU_CAR, \
                                  SECOC_CAR
from opendbc.sunnypilot.car.toyota.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type
SteerControlType = structs.CarParams.SteerControlType
AccelPersonality = custom.LongitudinalPlanSP.AccelerationPersonality

TEMP_STEER_FAULTS = (0, 9, 11, 21, 25)
PERM_STEER_FAULTS = (3, 17)


class CarState(CarStateBase, CarStateExt):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    CarStateExt.__init__(self, CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint])
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    if CP.flags & ToyotaFlags.SECOC.value:
      self.shifter_values = can_define.dv
    else:
      self.shifter_values = can_define.dv

    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    self.lkas_button = 0
    self.distance_button = 0

    self.pcm_follow_distance = 0

    self.acc_type = 1
    self.lkas_hud = {}
    self.gvc = 0.0
    self.secoc_synchronization = None

    self._left_blindspot = False
    self._left_blindspot_d1 = 0
    self._left_blindspot_d2 = 0
    self._left_blindspot_counter = 0

    self._right_blindspot = False
    self._right_blindspot_d1 = 0
    self._right_blindspot_d2 = 0
    self._right_blindspot_counter = 0

    self.signals_checked = False
    self.sport_signal_seen = False
    self.eco_signal_seen = False
    self.accel_profile = None
    self.prev_accel_profile = None
    self.accel_profile_init = False
    self.toyota_drive_mode = Params().get_bool('ToyotaDriveMode')

    # SUNNYPILOT MOD: Prev button states for forced +/- 5
    self.prev_res_plus = False
    self.prev_set_minus = False

    if CP_SP.flags & ToyotaFlagsSP.SP_AUTO_BRAKE_HOLD:
      self.pre_collision_2 = {}

    self.frame = 0

  def update(self, can_parsers) -> tuple:
    cp = can_parsers
    cp_cam = can_parsers

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()
    cp_acc = cp_cam if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) else cp

    if not self.CP.flags & ToyotaFlags.SECOC.value:
      self.gvc = cp.vl["GVC"]

    ret.doorOpen = any(, cp.vl,
                        cp.vl, cp.vl])
    ret.seatbeltUnlatched = cp.vl!= 0
    ret.parkingBrake = cp.vl == 1

    ret.brakePressed = cp.vl!= 0

    if self.CP.flags & ToyotaFlags.SECOC.value:
      self.secoc_synchronization = copy.copy(cp.vl)
      ret.gasPressed = cp.vl > 0
      can_gear = int(cp.vl)
    else:
      ret.gasPressed = cp.vl == 0
      can_gear = int(cp.vl)
      if not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
        ret.stockAeb = bool(cp_acc.vl and cp_acc.vl < -1e-5)

    if self.toyota_drive_mode:
      sport_signal = 'SPORT_ON_2' if self.CP.carFingerprint in (CAR.TOYOTA_RAV4_TSS2, CAR.LEXUS_ES_TSS2, CAR.TOYOTA_HIGHLANDER_TSS2) else 'SPORT_ON'
      if not self.signals_checked:
        self.signals_checked = True
        try:
          sport_mode = cp.vl[sport_signal]
          self.sport_signal_seen = True
        except KeyError:
          sport_mode = 0
          self.sport_signal_seen = False
        try:
          eco_mode = cp.vl['ECON_ON']
          self.eco_signal_seen = True
        except KeyError:
          eco_mode = 0
          self.eco_signal_seen = False
      else:
        sport_mode = cp.vl[sport_signal] if self.sport_signal_seen else 0
        eco_mode = cp.vl['ECON_ON'] if self.eco_signal_seen else 0

      if sport_mode == 1:
        self.accel_profile = AccelPersonality.sport
      elif eco_mode == 1:
        self.accel_profile = AccelPersonality.eco
      else:
        self.accel_profile = AccelPersonality.normal

      if not self.accel_profile_init or self.accel_profile!= self.prev_accel_profile:
        Params().put_nonblocking('AccelPersonality', int(self.accel_profile))
        self.accel_profile_init = True
        self.prev_accel_profile = self.accel_profile

    self.parse_wheel_speeds(ret,
      cp.vl,
      cp.vl,
      cp.vl,
      cp.vl,
    )
    ret.vEgoCluster = ret.vEgo * 1.015

    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    ret.steeringAngleDeg = cp.vl + cp.vl
    ret.steeringRateDeg = cp.vl
    torque_sensor_angle_deg = cp.vl

    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl):
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
        self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)
      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl == 1
    ret.rightBlinker = cp.vl == 2

    ret.steeringTorque = cp.vl
    ret.steeringTorqueEps = cp.vl * self.eps_torque_scale
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    ret.steerFaultTemporary = cp.vl in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = cp.vl in PERM_STEER_FAULTS

    if self.CP.steerControlType == SteerControlType.angle:
      ret.steerFaultTemporary = ret.steerFaultTemporary or cp.vl in TEMP_STEER_FAULTS
      ret.steerFaultPermanent = ret.steerFaultPermanent or cp.vl in PERM_STEER_FAULTS
      ret.vehicleSensorsInvalid = not self.accurate_steer_angle_seen

    if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      ret.cruiseState.available = cp.vl["MAIN_ON"]!= 0
      ret.cruiseState.speed = cp.vl * CV.KPH_TO_MS
      cluster_set_speed = cp.vl
    else:
      ret.accFaulted = cp.vl!= 0
      ret.carFaultedNonCritical = cp.vl!= 0
      ret.cruiseState.available = cp.vl["MAIN_ON"]!= 0
      ret.cruiseState.speed = cp.vl * CV.KPH_TO_MS
      cluster_set_speed = cp.vl

    is_metric = cp.vl in (1, 2)
    if ret.cruiseState.speed!= 0:
      conversion_factor = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
      ret.cruiseState.speedCluster = cluster_set_speed * conversion_factor

    if self.CP.carFingerprint in TSS2_CAR and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      self.acc_type = cp_acc.vl
      ret.stockFcw = bool(cp_acc.vl)

    if (self.CP.carFingerprint not in TSS2_CAR and self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR) or \
       (self.CP.carFingerprint in TSS2_CAR and self.acc_type == 1):
      if self.CP.openpilotLongitudinalControl:
        ret.accFaulted = ret.accFaulted or cp.vl == 2

    self.pcm_acc_status = cp.vl
    if self.CP.carFingerprint not in (NO_STOP_TIMER_CAR - TSS2_CAR):
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl)
    ret.cruiseState.nonAdaptive = self.pcm_acc_status in (1, 2, 3, 4, 5, 6)

    ret.genericToggle = bool(cp.vl)
    ret.espDisabled = cp.vl!= 0

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl == 1) or (cp.vl == 1)
      ret.rightBlindspot = (cp.vl == 1) or (cp.vl == 1)

    if self.CP.carFingerprint!= CAR.TOYOTA_PRIUS_V:
      self.lkas_hud = copy.copy(cp_cam.vl)

    if self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR:
      self.pcm_follow_distance = cp.vl

    buttonEvents =
    prev_distance_button = self.distance_button
    if self.CP.carFingerprint in TSS2_CAR:
      prev_lkas_button = self.lkas_button
      self.lkas_button = cp_cam.vl
      if self.lkas_button!= 0 and self.lkas_button!= prev_lkas_button:
        buttonEvents.extend(create_button_events(1, 0, {1: ButtonType.lkas}) +
                            create_button_events(0, 1, {1: ButtonType.lkas}))
      if self.CP.carFingerprint not in (RADAR_ACC_CAR | SECOC_CAR):
        self.distance_button = cp_acc.vl
        buttonEvents += create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})
    elif self.CP_SP.flags & ToyotaFlagsSP.SMART_DSU and not self.CP_SP.flags & ToyotaFlagsSP.RADAR_CAN_FILTER:
      self.distance_button = cp.vl
      buttonEvents += create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    ret.buttonEvents = buttonEvents

    # --- START SUNNYPILOT MOD: FORCED +/- 5 LOGIC ---
    # Delete standard +/- 1 events and replace with forced +/- 5 events
    try:
      res_plus = cp.vl
      set_minus = cp.vl
      ret.buttonEvents =
      ret.buttonEvents.extend(create_button_events(res_plus, self.prev_res_plus, {1: ButtonType.longAccelCruise}))
      ret.buttonEvents.extend(create_button_events(set_minus, self.prev_set_minus, {1: ButtonType.longDecelCruise}))
      self.prev_res_plus = res_plus
      self.prev_set_minus = set_minus
    except (KeyError, IndexError):
      pass
    # --- END SUNNYPILOT MOD ---

    if self.CP_SP.flags & ToyotaFlagsSP.SP_ENHANCED_BSM and self.frame > 199:
      ret.leftBlindspot, ret.rightBlindspot = self.sp_get_enhanced_bsm(cp)

    if self.CP_SP.flags & ToyotaFlagsSP.SP_AUTO_BRAKE_HOLD:
      self.pre_collision_2 = copy.copy(cp_cam.vl)

    self.frame += 1

    CarStateExt.update(self, ret, ret_sp, can_parsers)

    return ret, ret_sp

  def sp_get_enhanced_bsm(self, cp):
    distance_1 = cp.vl.get('BLINDSPOTD1')
    distance_2 = cp.vl.get('BLINDSPOTD2')
    side = cp.vl.get('BLINDSPOTSIDE')
    if all(val is not None for val in [distance_1, distance_2, side]):
      if side == 65:
        if distance_1!= self._left_blindspot_d1 or distance_2!= self._left_blindspot_d2:
          self._left_blindspot_d1 = distance_1
          self._left_blindspot_d2 = distance_2
          self._left_blindspot_counter = 100
        self._left_blindspot = distance_1 > 10 or distance_2 > 10
      elif side == 66:
        if distance_1!= self._right_blindspot_d1 or distance_2!= self._right_blindspot_d2:
          self._right_blindspot_d1 = distance_1
          self._right_blindspot_d2 = distance_2
          self._right_blindspot_counter = 100
        self._right_blindspot = distance_1 > 10 or distance_2 > 10
      self._left_blindspot_counter = max(0, self._left_blindspot_counter - 1)
      self._right_blindspot_counter = max(0, self._right_blindspot_counter - 1)
      if self._left_blindspot_counter == 0:
        self._left_blindspot = False
        self._left_blindspot_d1 = self._left_blindspot_d2 = 0
      if self._right_blindspot_counter == 0:
        self._right_blindspot = False
        self._right_blindspot_d1 = self._right_blindspot_d2 = 0
    return self._left_blindspot, self._right_blindspot

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages =
    if CP.flags & ToyotaFlags.SECOC.value:
      pt_messages.append(("GEAR_PACKET_HYBRID", 1))
      pt_messages.append(("SECOC_SYNCHRONIZATION", 10))

    cam_messages =

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint], cam_messages, 2),
    }
