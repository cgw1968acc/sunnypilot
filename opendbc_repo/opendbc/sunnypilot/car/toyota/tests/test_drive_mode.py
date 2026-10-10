import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.toyota.values import CAR
from opendbc.sunnypilot.car.toyota.drive_mode import get_drive_mode

AP = structs.CarStateSP.AccelerationPersonality
DBCS = ["toyota_nodsu_pt_generated", "toyota_new_mc_pt_generated", "toyota_tnga_k_pt_generated"]


def _cp(dbc, values):
  packer = CANPacker(dbc)
  cp = CANParser(dbc, [("GEAR_PACKET", 0)], 0)
  addr, dat, bus = packer.make_can_msg("GEAR_PACKET", 0, values)
  cp.update([(int(1e9), [(addr, dat, bus)])])
  return cp


def _CP(fingerprint):
  CP = structs.CarParams()
  CP.carFingerprint = fingerprint
  return CP


@pytest.mark.parametrize("dbc", DBCS)
class TestDriveMode:
  def test_corolla_bits(self, dbc):
    CP = _CP(CAR.TOYOTA_COROLLA_TSS2)
    assert get_drive_mode(CP, _cp(dbc, {"GEAR": 0})) == AP.normal
    assert get_drive_mode(CP, _cp(dbc, {"ECON_ON": 1})) == AP.eco
    assert get_drive_mode(CP, _cp(dbc, {"SPORT_ON": 1})) == AP.sport
    assert get_drive_mode(CP, _cp(dbc, {"SPORT_ON": 1, "ECON_ON": 1})) == AP.sport  # sport wins
    assert get_drive_mode(CP, _cp(dbc, {"SPORT_ON_2": 1})) == AP.normal  # bit 55 means nothing on this car

  def test_rav4_uses_bit_55(self, dbc):
    CP = _CP(CAR.TOYOTA_RAV4_TSS2)
    assert get_drive_mode(CP, _cp(dbc, {"SPORT_ON_2": 1})) == AP.sport
    assert get_drive_mode(CP, _cp(dbc, {"SPORT_ON": 1})) == AP.normal
    assert get_drive_mode(CP, _cp(dbc, {"ECON_ON": 1})) == AP.eco

  def test_new_signal_does_not_disturb_the_gear(self, dbc):
    cp = _cp(dbc, {"GEAR": 32, "SPORT_ON_2": 1, "ECON_ON": 1, "SPORT_ON": 0})
    assert cp.vl["GEAR_PACKET"]["GEAR"] == 32
    assert cp.vl["GEAR_PACKET"]["ECON_ON"] == 1
