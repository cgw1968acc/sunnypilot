# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
# Recorded reference only: nothing sends these any more (owner 2026-10-29). The Corolla Cross head unit (Toyota
# Drive+ Link) stops all its diagnostic polling as soon as another tester's frames appear, and stays stopped into the
# next ignition cycle: route 000000cb, our lock at 22:08 -> its last poll 4 ms earlier, app dead from 22:09; our unlock
# frames after ignition off (pandad, 54f6d5d/91b3ae7) -> app dead again on the next start (and from the camera harness
# the second frame blocked can_send 1.3-1.8 s, so the unlock never completed). The car locks itself above ~30 km/h;
# unlocking is to be set in the body ECU customization (Techstream) instead.
#
# Corolla Cross (tncr): the body ECU (diagnostic address 0x750, sub-address 0x40) answers 7F 30 11 (service not
# supported) to the older 40 05 30 11 00 80/40 frames: route 000000bb (10-28) and 000000be (10-29), every lock and
# unlock, so they never worked here. The owner's OBD window module locked the Cross on every drive of ~40 routes
# (09-25..10-28) with a session change and an IOControlByIdentifier on DID 0x2810, 50 ms apart, each answered
# positively (50 01 / 50 03 / 6F 28 10 03):
#   40 02 10 01 / 40 02 10 03 / 40 06 2F 28 10 03 01 80 (lock) / 40 06 2F 28 10 03 00 80 (return control)
# The window module unlocks at ignition off, when loggerd has stopped; recorded on 10-29 (pandad kept CAN live after
# the ignition went off, /data/media/0/unlock_capture/20261029-125722.txt): 1.28 s after ignition off the same
# session pair, then the same IOControl on DID 0x2811, each answered positively (6F 28 11 03):
#   40 06 2F 28 11 03 01 80 (unlock) / 40 06 2F 28 11 03 00 80 (return control)
# (the earlier guess, DID 0x2810 with mask 0x40, was refused: 7F 2F 31 requestOutOfRange, routes 000000c2/c3).
# The panda Toyota safety allows exactly these frames (opendbc/safety/modes/toyota.h sp_valid_uds_msgs).
SESSION_DEFAULT = b'\x40\x02\x10\x01\x00\x00\x00\x00'
SESSION_EXTENDED = b'\x40\x02\x10\x03\x00\x00\x00\x00'
LOCK_SEQ = (SESSION_DEFAULT, SESSION_EXTENDED, b'\x40\x06\x2f\x28\x10\x03\x01\x80', b'\x40\x06\x2f\x28\x10\x03\x00\x80')
UNLOCK_SEQ = (SESSION_DEFAULT, SESSION_EXTENDED, b'\x40\x06\x2f\x28\x11\x03\x01\x80', b'\x40\x06\x2f\x28\x11\x03\x00\x80')
