from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

FW_VERSIONS_EXT = {
}


# The firmware query alone is not a reliable hybrid detector: on the Corolla Altis Hybrid (comma 3X) the hybrid
# ECU (0x7e2) answered in only 23 of 42 boots in the 2026-09-27 backup and not on the 2026-09-29 08:12 drive,
# and every miss ran the car as a petrol Corolla (stopAccel -0.02, 0.15 s actuator delay, no engine state).
# The CAN broadcast set is deterministic: hybrids send GEAR_PACKET_HYBRID (0x127) and GAS_PEDAL_HYBRID (0x245)
# and never GAS_PEDAL (0x2C1); petrol cars send 0x2C1 and neither of the others (openpilot v0.8.5 CAN
# fingerprints: Prius, Camry/Corolla/RX hybrids vs Camry, Highlander, RAV4 TSS2, Lexus IS; the Altis matched the
# hybrid pattern in all 42 boots). A petrol car therefore cannot match this rule, and the flag is set when either
# source says hybrid.


def hybrid_can_messages(fingerprint: dict[int, dict[int, int]]) -> bool:
  """True when CAN carries the hybrid-only gear/gas messages and not the petrol gas pedal message.

  Called next to the stock firmware check."""
  bus0 = fingerprint.get(0, {})
  return 0x127 in bus0 and 0x245 in bus0 and 0x2C1 not in bus0
