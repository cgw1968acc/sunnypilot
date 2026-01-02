from openpilot.common.params import Params
from opendbc.car import Bus, structs, get_safety_config, uds
from.carstate import CarState
from.carcontroller import CarController
from.radar_interface import RadarInterface
from opendbc.car.toyota.values import Ecu, CAR, DBC, ToyotaFlags, CarControllerParams, TSS2_CAR, RADAR_ACC_CAR, NO_DSU_CAR, \
                                  MIN_ACC_SPEED, EPS_SCALE, NO_STOP_TIMER_CAR, ANGLE_CONTROL_CAR, \
                                  ToyotaSafetyFlags, UNSUPPORTED_DSU_CAR, SECOC_CAR
from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, ToyotaSafetyFlagsSP

SteerControlType = structs.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, CP_SP, current_speed, cruise_speed):
    return CarControllerParams(CP).ACCEL_MIN, CarControllerParams(CP).ACCEL_MAX

  def update(self, c, can_parsers):
    ret, ret_sp = super().update(c, can_parsers)
    for b in ret.buttonEvents:
      if b.type == structs.CarState.ButtonEvent.Type.accelCruise:
        b.type = structs.CarState.ButtonEvent.Type.longAccelCruise 
      elif b.type == structs.CarState.ButtonEvent.Type.decelCruise:
        b.type = structs.CarState.ButtonEvent.Type.longDecelCruise 
    return ret, ret_sp

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "toyota"
    ret.safetyConfigs =
    ret.safetyConfigs.safetyParam = EPS_SCALE[candidate]

    if DBC[candidate] == "toyota_new_mc_pt_generated":
      ret.safetyConfigs.safetyParam |= ToyotaSafetyFlags.ALT_BRAKE.value

    if ret.flags & ToyotaFlags.SECOC.value:
      ret.secOcRequired = True
      ret.safetyConfigs.safetyParam |= ToyotaSafetyFlags.SECOC.value
      ret.dashcamOnly = is_release

    if candidate in ANGLE_CONTROL_CAR:
      ret.steerControlType = SteerControlType.angle
      ret.safetyConfigs.safetyParam |= ToyotaSafetyFlags.LTA.value
      ret.steerActuatorDelay = 0.18
      ret.steerLimitTimer = 0.8
    else:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerActuatorDelay = 0.12
      ret.steerLimitTimer = 0.4

    stop_and_go = candidate in TSS2_CAR
    found_ecus = [fw.ecu for fw in car_fw]
    if Ecu.hybrid in found_ecus:
      ret.flags |= ToyotaFlags.HYBRID.value

    if candidate == CAR.TOYOTA_PRIUS:
      stop_and_go = True
      for fw in car_fw:
        if fw.ecu == "eps" and not fw.fwVersion == b'8965B47060\x00\x00\x00\x00\x00\x00':
          ret.steerActuatorDelay = 0.25
          CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.2)
    elif candidate in (CAR.LEXUS_RX, CAR.LEXUS_RX_TSS2):
      stop_and_go = True
      ret.wheelSpeedFactor = 1.035
    elif candidate in (CAR.TOYOTA_AVALON, CAR.TOYOTA_AVALON_2019, CAR.TOYOTA_AVALON_TSS2):
      stop_and_go = candidate!= CAR.TOYOTA_AVALON
    elif candidate in (CAR.TOYOTA_RAV4_TSS2, CAR.TOYOTA_RAV4_TSS2_2022, CAR.TOYOTA_RAV4_TSS2_2023, CAR.TOYOTA_RAV4_PRIME, CAR.TOYOTA_SIENNA_4TH_GEN):
      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kiBP = [0.0]
      ret.lateralTuning.pid.kpBP = [0.0]
      ret.lateralTuning.pid.kpV = [0.6]
      ret.lateralTuning.pid.kiV = [0.1]
      ret.lateralTuning.pid.kf = 0.00007818594
      for fw in car_fw:
        if fw.ecu == "eps" and (fw.fwVersion.startswith(b'\x02') or fw.fwVersion in):
          ret.lateralTuning.pid.kpV = [0.15]
          ret.lateralTuning.pid.kiV = [0.05]
          ret.lateralTuning.pid.kf = 0.00004
          break
    elif candidate in (CAR.TOYOTA_CHR, CAR.TOYOTA_CAMRY, CAR.TOYOTA_SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_NX):
      stop_and_go = True

    ret.centerToFront = ret.wheelbase * 0.44
    ret.enableBsm = 0x3F6 in fingerprint and candidate in TSS2_CAR
    ret.radarUnavailable = Bus.radar not in DBC[candidate] or candidate in (NO_DSU_CAR - TSS2_CAR)

    if candidate in (RADAR_ACC_CAR | NO_DSU_CAR):
      ret.alphaLongitudinalAvailable = candidate in RADAR_ACC_CAR
      if alpha_long and candidate in RADAR_ACC_CAR:
        ret.flags |= ToyotaFlags.DISABLE_RADAR.value

    ret.openpilotLongitudinalControl = (candidate in (TSS2_CAR - RADAR_ACC_CAR) or
                                        bool(ret.flags & ToyotaFlags.DISABLE_RADAR.value))
    ret.autoResumeSng = ret.openpilotLongitudinalControl and candidate in NO_STOP_TIMER_CAR

    if not ret.openpilotLongitudinalControl:
      ret.safetyConfigs.safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    ret.minEnableSpeed = -1. if stop_and_go else MIN_ACC_SPEED
    sp_tss2_long_tune = Params().get_bool("ToyotaTSS2Long")

    if candidate in TSS2_CAR:
      ret.flags |= ToyotaFlags.RAISED_ACCEL_LIMIT.value
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.01
      ret.stoppingDecelRate = 0.03 if sp_tss2_long_tune else 0.3
      if ret.flags & ToyotaFlags.HYBRID.value:
        ret.longitudinalActuatorDelay = 0.05

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                      car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    if candidate in UNSUPPORTED_DSU_CAR:
      ret.safetyParam |= ToyotaSafetyFlagsSP.UNSUPPORTED_DSU

    sp_toyota_auto_brake_hold = Params().get_bool("ToyotaAutoHold")
    sp_toyota_enhanced_bsm = Params().get_bool("ToyotaEnhancedBsm")
    if sp_toyota_enhanced_bsm and candidate in (TSS2_CAR - SECOC_CAR):
      ret.flags |= ToyotaFlagsSP.SP_ENHANCED_BSM.value
    if candidate == CAR.TOYOTA_PRIUS_TSS2:
      ret.flags |= ToyotaFlagsSP.SP_NEED_DEBUG_BSM.value
    if sp_toyota_auto_brake_hold and candidate in (TSS2_CAR - RADAR_ACC_CAR - SECOC_CAR):
      ret.flags |= ToyotaFlagsSP.SP_AUTO_BRAKE_HOLD.value

    if 0x2FF in fingerprint or (0x2AA in fingerprint and candidate in NO_DSU_CAR):
      ret.flags |= ToyotaFlagsSP.SMART_DSU.value
    if 0x2AA in fingerprint and candidate in NO_DSU_CAR:
      ret.flags |= ToyotaFlagsSP.RADAR_CAN_FILTER.value
    if 0x23 in fingerprint and not stock_cp.flags & ToyotaFlags.SECOC.value:
      ret.flags |= ToyotaFlagsSP.ZSS.value

    if candidate == CAR.TOYOTA_PRIUS:
      if ret.flags & ToyotaFlagsSP.ZSS.value:
        stock_cp.steerRatio = 15.0
        stock_cp.mass = 3370.
        for fw in car_fw:
          if fw.ecu == "eps" and not fw.fwVersion == b'8965B47060\x00\x00\x00\x00\x00\x00':
            stock_cp.steerActuatorDelay = 0.25
            CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning, steering_angle_deadzone_deg=0.0)

    use_sdsu = bool(ret.flags & ToyotaFlagsSP.SMART_DSU.value)
    stock_cp.minEnableSpeed = -1. if use_sdsu else stock_cp.minEnableSpeed

    if candidate in (RADAR_ACC_CAR | NO_DSU_CAR):
      stock_cp.alphaLongitudinalAvailable = use_sdsu or candidate in RADAR_ACC_CAR
      if not use_sdsu:
        if alpha_long and candidate in RADAR_ACC_CAR:
          stock_cp.flags |= ToyotaFlags.DISABLE_RADAR.value
      else:
        use_sdsu = use_sdsu and alpha_long

    stock_cp.openpilotLongitudinalControl = use_sdsu or \
      candidate in (TSS2_CAR - RADAR_ACC_CAR) or \
      bool(stock_cp.flags & ToyotaFlags.DISABLE_RADAR.value)

    ret.enableGasInterceptor = 0x201 in fingerprint and stock_cp.openpilotLongitudinalControl and \
                               not stock_cp.flags & ToyotaFlags.SECOC.value

    if ret.enableGasInterceptor:
      ret.safetyParam |= ToyotaSafetyFlagsSP.GAS_INTERCEPTOR
      stock_cp.minEnableSpeed = -1.

    if ret.flags & ToyotaFlagsSP.STOCK_LONGITUDINAL.value:
      stock_cp.alphaLongitudinalAvailable = False
      stock_cp.openpilotLongitudinalControl = False

    if not stock_cp.openpilotLongitudinalControl:
      stock_cp.safetyConfigs.safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value
    else:
      stock_cp.safetyConfigs.safetyParam &= ~ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    return ret

  @staticmethod
  def init(CP, CP_SP, can_recv, can_send, communication_control=None):
    if CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if communication_control is None:
        communication_control = bytes()
      disable_ecu(can_recv, can_send, bus=0, addr=0x750, sub_addr=0xf, com_cont_req=communication_control)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    communication_control = bytes()
    CarInterface.init(CP, None, can_recv, can_send, communication_control)
