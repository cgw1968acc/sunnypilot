#!/usr/bin/env python3
"""
tncr, temporary (owner 2026-10-29): record the OBD window module's door unlock. The module unlocks the Corolla Cross
the moment the ignition goes off, after loggerd has stopped; pandad keeps CAN live for UNLOCK_CAPTURE_MS after the
ignition goes off (pandad.cc) and this writes every diagnostic frame (0x700-0x7ff) from PRE_S before to POST_S after
it to /data/media/0/unlock_capture/. Until then 'can' is only buffered as raw bytes (never parsed).
"""
import collections
import os
import time

from openpilot.cereal import messaging

OUT_DIR = "/data/media/0/unlock_capture"
PRE_S = 3.0
POST_S = 20.0


def frames(raw):
  evt = messaging.log_from_bytes(raw)
  return [(evt.logMonoTime, c.src, c.address, bytes(c.dat)) for c in evt.can if 0x700 <= c.address <= 0x7ff]


def main():
  can_sock = messaging.sub_sock("can", conflate=False, timeout=100)
  ps_sock = messaging.sub_sock("pandaStates", conflate=True)
  buf = collections.deque()
  ignition = None
  capture_until = None
  out = None
  while True:
    for raw in messaging.drain_sock_raw(can_sock, wait_for_one=True):
      now = time.monotonic()
      if out is not None:
        for t, src, addr, dat in frames(raw):
          out.write(f"{t} {src} {addr:#x} {dat.hex()}\n")
      else:
        buf.append((now, raw))
        while buf and now - buf[0][0] > PRE_S:
          buf.popleft()

    ps = messaging.recv_one_or_none(ps_sock)
    if ps is not None and len(ps.pandaStates):
      ign = ps.pandaStates[0].ignitionLine or ps.pandaStates[0].ignitionCan
      if ignition and not ign and out is None:
        os.makedirs(OUT_DIR, exist_ok=True)
        out = open(os.path.join(OUT_DIR, time.strftime("%Y%m%d-%H%M%S") + ".txt"), "w")
        out.write(f"# ignition off, wall {time.clock_gettime(time.CLOCK_REALTIME):.3f}, boot_ns {time.clock_gettime_ns(time.CLOCK_BOOTTIME)}\n")
        for _, raw in buf:
          for t, src, addr, dat in frames(raw):
            out.write(f"{t} {src} {addr:#x} {dat.hex()}\n")
        buf.clear()
        capture_until = time.monotonic() + POST_S
      ignition = ign

    if out is not None and time.monotonic() > capture_until:
      out.close()
      out = None


if __name__ == "__main__":
  main()
