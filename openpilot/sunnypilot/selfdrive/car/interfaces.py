"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from typing import Any

from opendbc.car import structs
from opendbc.car.interfaces import CarInterfaceBase
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.helpers import get_nn_model_path
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import set_speed_limit_assist_availability

import openpilot.system.sentry as sentry

from openpilot.sunnypilot.sunnylink.statsd import STATSLOGSP


def cached_car_params(params: Params) -> bytes | None:
  """
  CarParams to fingerprint from without the VIN/firmware query (opendbc car_helpers.fingerprint uses them when they
  hold carFw and a VIN). Stock only has CarParamsCache, cleared on every manager start, so every cold boot queries
  ~60 ECUs over diagnostics ~2 s after start.
  Why it matters (tncr 2026-10-29, Corolla Cross hybrid, dealer CarMate head unit with Toyota Drive+ Link): the head
  unit polls the car's ECUs over UDS all through the ignition cycle and feeds the Drive+ Link app. It stops polling
  the moment another tester's diagnostic frames appear on the bus and stays stopped into the next ignition cycle, so
  the app's vehicle status freezes. Routes 000000bd/000000be: last poll at 6.3 s, right in this query; route 000000cb:
  with no query and nothing else sent, polling ran the whole drive (~1000 reads/min) and the app kept updating.
  When the platform is already fixed by the user (CarPlatformBundle), the identity cannot come from the query anyway,
  so the last drive's CarParams (CarParamsPersistent, same carFw and VIN) are used and the query is skipped. Without a
  fixed platform, or on the very first boot (no CarParamsPersistent yet), the stock query runs unchanged.
  """
  cached = params.get("CarParamsCache")
  if cached is not None:
    return cached
  if (params.get("CarPlatformBundle") or {}).get("platform") is None:
    return None
  return params.get("CarParamsPersistent")


def log_fingerprint(CP: structs.CarParams) -> None:
  if CP.carFingerprint == "MOCK":
    sentry.capture_fingerprint_mock()
  else:
    sentry.capture_fingerprint(CP.carFingerprint, CP.brand)


def _enforce_torque_lateral_control(CP: structs.CarParams, params: Params | None = None, enabled: bool = False) -> bool:
  if params is None:
    params = Params()

  if CP.steerControlType != structs.CarParams.SteerControlType.angle:
    enabled = params.get_bool("EnforceTorqueControl")

  return enabled


def _initialize_neural_network_lateral_control(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                               params: Params | None = None, enabled: bool = False) -> bool:
  if params is None:
    params = Params()

  nnlc_model_path, nnlc_model_name, exact_match = get_nn_model_path(CP)

  if nnlc_model_name == "MOCK":
    cloudlog.error({"nnlc event": "car doesn't match any Neural Network model"})

  if nnlc_model_name != "MOCK" and CP.steerControlType != structs.CarParams.SteerControlType.angle:
    enabled = params.get_bool("NeuralNetworkLateralControl")

  CP_SP.neuralNetworkLateralControl.model.path = nnlc_model_path
  CP_SP.neuralNetworkLateralControl.model.name = nnlc_model_name
  CP_SP.neuralNetworkLateralControl.fuzzyFingerprint = not exact_match

  return enabled


def _initialize_intelligent_cruise_button_management(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params | None = None) -> None:
  if params is None:
    params = Params()

  icbm_enabled = params.get_bool("IntelligentCruiseButtonManagement")
  if icbm_enabled and CP_SP.intelligentCruiseButtonManagementAvailable and not CP.openpilotLongitudinalControl:
    CP_SP.pcmCruiseSpeed = False


def _initialize_torque_lateral_control(CI: CarInterfaceBase, CP: structs.CarParams, enforce_torque: bool, nnlc_enabled: bool) -> None:
  if nnlc_enabled or enforce_torque:
    CI.configure_torque_tune(CP.carFingerprint, CP.lateralTuning)


def _cleanup_unsupported_params(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params | None = None) -> None:
  if params is None:
    params = Params()

  if params.get_bool("LateralJerkTorqueController") and params.get_bool("NeuralNetworkLateralControl"):
    cloudlog.warning("LateralJerkTorqueController and NeuralNetworkLateralControl both enabled, disabling both")
    params.put_bool("LateralJerkTorqueController", False, block=True)
    params.put_bool("NeuralNetworkLateralControl", False, block=True)

  if CP.steerControlType == structs.CarParams.SteerControlType.angle:
    cloudlog.warning("SteerControlType is angle, cleaning up params")
    params.remove("NeuralNetworkLateralControl")
    params.remove("EnforceTorqueControl")
    params.remove("LateralJerkTorqueController")

  if not CP_SP.intelligentCruiseButtonManagementAvailable or CP.openpilotLongitudinalControl:
    cloudlog.warning("ICBM not available or openpilot Longitudinal Control enabled, cleaning up params")
    params.remove("IntelligentCruiseButtonManagement")

  if not CP.openpilotLongitudinalControl and CP_SP.pcmCruiseSpeed:
    cloudlog.warning("openpilot Longitudinal Control and ICBM not available, cleaning up params")
    params.remove("DynamicExperimentalControl")
    params.remove("CustomAccIncrementsEnabled")
    params.remove("SmartCruiseControlVision")
    params.remove("SmartCruiseControlMap")

  set_speed_limit_assist_availability(CP, CP_SP, params)


def setup_interfaces(CI: CarInterfaceBase, params: Params | None = None) -> None:
  enforce_torque = _enforce_torque_lateral_control(CI.CP, params)
  nnlc_enabled = _initialize_neural_network_lateral_control(CI.CP, CI.CP_SP, params)
  _initialize_intelligent_cruise_button_management(CI.CP, CI.CP_SP, params)
  _initialize_torque_lateral_control(CI, CI.CP, enforce_torque, nnlc_enabled)
  _cleanup_unsupported_params(CI.CP, CI.CP_SP)

  try:
    STATSLOGSP.raw('sunnypilot.car_params', CI.CP.to_dict())
  except RuntimeError:
    pass  # to_dict fails on macOS due to library issues.
  # STATSLOGSP.raw('sunnypilot_params.car_params_sp', CP_SP.to_dict()) # https://github.com/sunnypilot/opendbc/pull/361


def initialize_params(params) -> list[dict[str, Any]]:
  keys: list = []

  # hyundai
  keys.extend([
    "HyundaiLongitudinalTuning",
  ])

  # subaru
  keys.extend([
    "SubaruStopAndGo",
    "SubaruStopAndGoManualParkingBrake",
  ])

  # tesla
  keys.extend([
    "TeslaCoopSteering",
    "TeslaMadsScreenButton",
  ])

  # toyota
  keys.extend([
    "ToyotaEnforceStockLongitudinal",
    "ToyotaStopAndGoHack",
    "ToyotaTSS2Long",
    "ToyotaEnhancedBsm",
    "ToyotaAutoHold",
    "ToyotaBrakeOnset",
  ])

  return [{k: params.get(k, return_default=True)} for k in keys]
