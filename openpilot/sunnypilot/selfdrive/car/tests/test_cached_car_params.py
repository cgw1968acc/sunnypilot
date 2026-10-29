# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md).
from unittest import mock

from opendbc.car import structs
from opendbc.car.car_helpers import fingerprint
from opendbc.car.structs import car
from openpilot.sunnypilot.selfdrive.car.interfaces import cached_car_params


class FakeParams(dict):
  def get(self, key, block=False):
    return dict.get(self, key)


def _cp_bytes():
  CP = car.CarParams.new_message()
  CP.brand = "toyota"
  CP.carFingerprint = "TOYOTA_COROLLA_TSS2"
  CP.carVin = "RKLKCAAG3T0381660"
  fw = CP.init("carFw", 1)
  fw[0].ecu = "eps"
  fw[0].fwVersion = b"8965B12345\x00\x00\x00\x00\x00\x00"
  fw[0].address = 0x7a1
  fw[0].brand = "toyota"
  return CP.to_bytes()


def test_cache_first_then_persistent_only_with_fixed_platform():
  persistent = _cp_bytes()
  assert cached_car_params(FakeParams(CarParamsCache=b"c", CarParamsPersistent=persistent)) == b"c"
  assert cached_car_params(FakeParams(CarParamsPersistent=persistent)) is None   # platform not fixed: query as stock
  fixed = FakeParams(CarParamsPersistent=persistent, CarPlatformBundle={"platform": "TOYOTA_COROLLA_TSS2"})
  assert cached_car_params(fixed) == persistent
  assert cached_car_params(FakeParams(CarPlatformBundle={"platform": "TOYOTA_COROLLA_TSS2"})) is None  # first boot


def test_fingerprint_from_persistent_sends_nothing():
  sent, obd = [], []
  raw = cached_car_params(FakeParams(CarParamsPersistent=_cp_bytes(), CarPlatformBundle={"platform": "TOYOTA_COROLLA_TSS2"}))
  with car.CarParams.from_bytes(raw) as cached, mock.patch("opendbc.car.car_helpers.can_fingerprint", return_value=(None, {})):
    fp = fingerprint(lambda wait_for_one=False: [], lambda msgs: sent.extend(msgs), lambda on: obd.append(on),
                     cached, "TOYOTA_COROLLA_TSS2")
  assert sent == []              # no VIN, no ECU presence, no firmware query
  assert True not in obd         # OBD multiplexing never switched on
  assert fp[2] == "RKLKCAAG3T0381660" and len(fp[3]) == 1 and fp[4] == structs.CarParams.FingerprintSource.fixed
