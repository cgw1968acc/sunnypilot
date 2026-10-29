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
# The doors are unlocked after the ignition goes off, the way the owner's OBD window module did it (owner 2026-10-29:
# "unlocking right after ignition off is ideal, I want the C4 to do that"): carcontroller no longer runs then, so
# pandad sends UNLOCK_SEQ (selfdrive/pandad/pandad.cc door_unlock_after_ignition, same bytes) while the Toyota safety
# is kept past the ignition off. Nothing is sent while driving any more (owner 2026-10-29): route 000000cb showed the
# head unit (Toyota Drive+ Link) stop all its diagnostic polling for the rest of the ignition cycle 4 ms after our lock
# sequence went out at 22:08 (it sees body ECU replies it did not ask for), and the app stopped updating; the car
# locks itself above walking speed without it. LOCK_SEQ is kept as the recorded reference (panda safety allows it).
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
