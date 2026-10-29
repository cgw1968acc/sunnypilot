# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from opendbc.sunnypilot.car.toyota.door_unlock import LOCK_SEQ, UNLOCK_SEQ


def le(b, i):
  return int.from_bytes(b[i:i + 4], 'little')


def test_bytes_match_panda_safety():
  # toyota.h sp_valid_uds_msgs: 40 02 10 01|03 (+ zeros) and 40 06 2F 28 | 10|11 03 0x 80
  assert [(le(m, 0), le(m, 4)) for m in LOCK_SEQ] == [(0x01100240, 0), (0x03100240, 0), (0x282F0640, 0x80010310), (0x282F0640, 0x80000310)]
  assert [(le(m, 0), le(m, 4)) for m in UNLOCK_SEQ] == [(0x01100240, 0), (0x03100240, 0), (0x282F0640, 0x80010311), (0x282F0640, 0x80000311)]
