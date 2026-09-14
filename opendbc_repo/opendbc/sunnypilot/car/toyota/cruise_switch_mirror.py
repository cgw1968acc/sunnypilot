"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Cruise switch mirror for Toyota TSS2 cars where openpilot owns the cruise set speed (pcmCruiseSpeed = False).

The PCM's own set speed, which is what the instrument cluster displays, only moves on physical RES/SET presses.
The raw switch state is published on 0x361 (byte 0: bit 3 CANCEL, bit 4 SET/-, bit 5 RES/+, bit 2 MAIN, bit 1
ENGAGED) at ~15.5 Hz on bus 0 (option1b_findings6/9). This module re-sends that frame with one press bit added so
the PCM's set speed can follow openpilot's. The panda only passes 0x361 when ToyotaSafetyFlagsSP.CRUISE_SWITCH_TX is
set, and only RES/SET while controls are allowed (opendbc/safety/modes/toyota.h).

Stage 2 (this file): a one-shot experiment. Touching TRIGGER_FILE on the device while cruise is engaged sends a
single burst that mimics one physical press, then writes RESULT_FILE. Whether the PCM listens to 0x361 at all is
still unknown (option1b_findings7 Q2, findings14); this is the test that answers it.
"""
import os
import time
from collections import deque

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.sunnypilot.car.toyota.values import ToyotaSafetyFlagsSP

CRUISE_SWITCH_ADDR = 0x361
CRUISE_SWITCH_BUS = 0
CRUISE_SWITCH_LEN = 8

BTN_CANCEL = 0x08
BTN_SET = 0x10
BTN_RES = 0x20
PRESS_MASK = BTN_CANCEL | BTN_SET | BTN_RES
BUTTONS = {"res": BTN_RES, "set": BTN_SET}

# findings9: a physical press holds the bit for 450-520 ms = 7-8 consecutive genuine frames at 64.5 ms
BURST_FRAMES = 8
# 100 Hz control frames after a genuine frame before the mirrored one goes out, ~30 ms = mid gap (findings10)
SEND_DELAY_FRAMES = 3
# the genuine template must be at most this old when the mirrored frame is sent (< 20 ms, findings8 freshness gate)
TEMPLATE_MAX_AGE_FRAMES = SEND_DELAY_FRAMES
# the last N genuine frames must carry an identical byte 0 (findings8 stability gate)
STABLE_FRAMES = 8
# give up if a burst has not completed within this many control frames (2 s)
BURST_TIMEOUT_FRAMES = 200
# how often the trigger file is polled while idle
TRIGGER_POLL_FRAMES = 100

TRIGGER_FILE = "/data/cruise_switch_burst"
RESULT_FILE = "/data/cruise_switch_burst.result"


class CruiseSwitchRaw:
  """Latest genuine 0x361 frames, captured from the raw CAN packets before parsing (bus 0 only, so openpilot's own
  transmissions echoed back by the panda on bus 128 are never mistaken for the stock stream)."""

  def __init__(self):
    self.frames: deque[bytes] = deque(maxlen=STABLE_FRAMES)
    self.seq = 0  # increments on every genuine frame, lets the controller detect a new one

  def update(self, can_packets: list[tuple[int, list[CanData]]]) -> None:
    for _, msgs in can_packets:
      for msg in msgs:
        if msg.src == CRUISE_SWITCH_BUS and msg.address == CRUISE_SWITCH_ADDR and len(msg.dat) == CRUISE_SWITCH_LEN:
          self.frames.append(bytes(msg.dat))
          self.seq += 1

  @property
  def latest(self) -> bytes | None:
    return self.frames[-1] if self.frames else None

  @property
  def stable(self) -> bool:
    return len(self.frames) == STABLE_FRAMES and len({f[0] for f in self.frames}) == 1


class CruiseSwitchMirrorCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.trigger_file = TRIGGER_FILE
    self.result_file = RESULT_FILE

    self.armed = False
    self.button = 0
    self.button_name = ""
    self.armed_frame = 0
    self.sent = 0
    self.last_seq = 0
    self.send_frame: int | None = None
    self.template: bytes | None = None
    self.template_frame = 0
    self.log: list[str] = []

  @property
  def enabled(self) -> bool:
    return (not self.CP_SP.pcmCruiseSpeed) and bool(self.CP_SP.safetyParam & ToyotaSafetyFlagsSP.CRUISE_SWITCH_TX)

  # ----- one-shot trigger -----

  def _poll_trigger(self) -> None:
    try:
      if not os.path.exists(self.trigger_file):
        return
      with open(self.trigger_file) as f:
        name = f.read().strip().lower() or "res"
      os.remove(self.trigger_file)
    except OSError:
      return

    if name not in BUTTONS:
      self._write_result(f"ignored trigger, unknown button {name!r} (use res or set)")
      return

    self.button_name = name
    self.button = BUTTONS[name]

  def _arm(self, frame: int, raw: "CruiseSwitchRaw") -> None:
    self.armed = True
    self.armed_frame = frame
    self.sent = 0
    self.send_frame = None
    self.template = None
    self.last_seq = raw.seq  # only genuine frames received after arming may become templates
    self.log = [f"armed {self.button_name} burst at frame {frame}"]
    carlog.warning(f"cruise switch mirror: {self.log[-1]}")

  def _finish(self, reason: str) -> None:
    self.log.append(reason)
    carlog.warning(f"cruise switch mirror: {reason}")
    self._write_result("\n".join(self.log))
    self.armed = False
    self.button = 0
    self.button_name = ""

  def _write_result(self, text: str) -> None:
    try:
      with open(self.result_file, "w") as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} monotonic={time.monotonic():.3f}\n{text}\n")
    except OSError:
      pass

  # ----- burst -----

  def update(self, CS, frame: int, cruise_engaged: bool) -> list[CanData]:
    if not self.enabled:
      return []

    raw: CruiseSwitchRaw = CS.cruise_switch

    if not self.armed:
      if frame % TRIGGER_POLL_FRAMES == 0:
        self._poll_trigger()
        if self.button:
          self._arm(frame, raw)
      if not self.armed:
        return []

    # abort conditions checked on every frame while armed
    if not cruise_engaged:
      self._finish(f"abort: cruise not engaged after {self.sent} frames")
      return []
    if frame - self.armed_frame > BURST_TIMEOUT_FRAMES:
      self._finish(f"abort: timeout after {self.sent} frames")
      return []

    # a new genuine frame schedules exactly one mirrored frame SEND_DELAY_FRAMES later
    if raw.seq != self.last_seq:
      self.last_seq = raw.seq
      self.template = raw.latest
      self.template_frame = frame
      self.send_frame = frame + SEND_DELAY_FRAMES

    if self.send_frame is None or frame != self.send_frame:
      return []
    self.send_frame = None

    template = self.template
    if template is None or frame - self.template_frame > TEMPLATE_MAX_AGE_FRAMES:
      self._finish(f"abort: stale template after {self.sent} frames")
      return []
    if not raw.stable:
      self._finish(f"abort: byte 0 not stable over the last {STABLE_FRAMES} genuine frames after {self.sent} frames")
      return []
    if template[0] & PRESS_MASK:
      self._finish(f"abort: driver is pressing a switch (byte0={template[0]:#04x}) after {self.sent} frames")
      return []

    out = bytes([template[0] | self.button]) + template[1:]
    assert out[0] == (template[0] | self.button) and out != template and len(out) == CRUISE_SWITCH_LEN

    self.sent += 1
    self.log.append(f"frame {frame}: sent {out.hex()} (template {template.hex()}) {self.sent}/{BURST_FRAMES}")
    can_sends = [CanData(CRUISE_SWITCH_ADDR, out, CRUISE_SWITCH_BUS)]

    if self.sent >= BURST_FRAMES:
      self._finish(f"done: {self.sent} {self.button_name} frames sent, check 0x1D3 SET_SPEED / cluster")

    return can_sends
