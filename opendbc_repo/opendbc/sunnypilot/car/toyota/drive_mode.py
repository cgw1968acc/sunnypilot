"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.car.toyota.values import CAR

AccelPersonality = structs.CarStateSP.AccelerationPersonality

# TN (ported from tnc3C): the car's drive mode switch picks the acceleration personality. These cars report SPORT on
# GEAR_PACKET bit 55 (SPORT_ON_2) instead of bit 2 (SPORT_ON).
SPORT_ON_2_CARS = (CAR.TOYOTA_RAV4_TSS2, CAR.LEXUS_ES_TSS2, CAR.TOYOTA_HIGHLANDER_TSS2)


def get_drive_mode(CP: structs.CarParams, cp) -> AccelPersonality:
  """SPORT beats ECO; anything else (or a car whose DBC lacks the signals) = normal. Not for SecOC cars (no GEAR_PACKET)."""
  gear_packet = cp.vl["GEAR_PACKET"]
  sport_signal = "SPORT_ON_2" if CP.carFingerprint in SPORT_ON_2_CARS else "SPORT_ON"
  if gear_packet.get(sport_signal, 0) == 1:
    return AccelPersonality.sport
  if gear_packet.get("ECON_ON", 0) == 1:
    return AccelPersonality.eco
  return AccelPersonality.normal
