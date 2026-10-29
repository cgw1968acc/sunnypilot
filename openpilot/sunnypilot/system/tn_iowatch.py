#!/usr/bin/env python3
# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Storage-stall recorder (tncr, owner 2026-10-22, route 0000009d seg 10 15:40:14). A 0.25 s storage stall froze
modeld (it sat in D state, waiting on the disk), 3 model frames were dropped, frameDropPerc reached 1.47 % and the
stock modeldLagging rule soft-disabled while pulling away from a light. procLog only showed the system-wide iowait jump
(0.1 % -> 8.9 %), not who was writing, and the kernel log of that boot was gone after the next reboot.

This process names the writer the next time it happens. It only reads /proc (no disk I/O of its own):
  - every SAMPLE_DT it reads the system iowait share from /proc/stat;
  - every SNAP_DT it keeps a snapshot of each readable process's /proc/<pid>/io counters (write_bytes, read_bytes,
    cancelled_write_bytes = deletes / truncates), plus /proc/diskstats and Dirty/Writeback from /proc/meminfo;
  - when iowait over one sample exceeds STALL_IOWAIT it takes a fresh snapshot and logs, through cloudlog (into the
    rlog's logMessage), the top TOP_N processes by bytes moved since the snapshot ~WINDOW s ago, every process and
    kernel thread in D state (a full /proc scan, only then),
    the disk's sectors written / read and busy time, and the dirty / writeback page cache. At most one record per
    MIN_GAP s.
Only processes of this user are snapshotted (root ones - kernel threads such as jbd2, sudo children - cannot be read as
the comma user, and skipping them takes a snapshot on the C4 from 89 ms to a few ms); the disk totals and the D-state
list still show them. ENABLED = False switches it off without touching the process list.
"""
import os
import time

from openpilot.common.swaglog import cloudlog

ENABLED = True
SAMPLE_DT = 0.25     # s between iowait samples
SNAP_DT = 1.0        # s between per-process snapshots
WINDOW = 2.0         # s the deltas are taken over (the oldest kept snapshot at least this old)
STALL_IOWAIT = 0.05  # iowait share over one sample that counts as a stall (normal driving ~0.001)
MIN_GAP = 5.0        # s between two records
TOP_N = 6
DISK = "sda"
KMSG_AFTER = 5.0     # s of continuous stall before the kernel log is captured
KMSG_WINDOW = 120.0  # s of kernel log kept
KMSG_REARM = 10.0    # s without a stall before the next stall may capture again
KMSG_MAX = 40        # lines per record
KMSG_KEYS = ("ufs", "scsi", "sd ", "sda", "blk", "mmc", "ext4", "jbd2", "i/o error", "timeout", "timed out", "hung_task",
             "blocked for more than", "reset", "abort", "err")


def read_cpu() -> tuple[int, int]:
  """(iowait, total) jiffies over all CPUs"""
  with open("/proc/stat") as f:
    v = [int(x) for x in f.readline().split()[1:]]
  return v[4], sum(v[:8])


def read_proc(pid: str) -> tuple[str, str, int, int, int] | None:
  """(comm, state, read_bytes, write_bytes, cancelled_write_bytes), None when gone; io is None-filled when unreadable"""
  try:
    with open(f"/proc/{pid}/stat") as f:
      st = f.read()
    comm = st[st.index("(") + 1:st.rindex(")")]
    state = st[st.rindex(")") + 2]
  except (OSError, ValueError):
    return None
  try:
    with open(f"/proc/{pid}/io") as f:
      io = dict(line.split(": ") for line in f.read().splitlines())
    return comm, state, int(io["read_bytes"]), int(io["write_bytes"]), int(io["cancelled_write_bytes"])
  except (OSError, KeyError, ValueError):
    return comm, state, -1, -1, -1


def read_disk(name: str = DISK) -> tuple[int, int, int]:
  """(sectors read, sectors written, ms busy)"""
  with open("/proc/diskstats") as f:
    for line in f:
      p = line.split()
      if p[2] == name:
        return int(p[5]), int(p[9]), int(p[12])
  return 0, 0, 0


def read_dirty() -> tuple[int, int]:
  """(Dirty, Writeback) kB"""
  d = {}
  with open("/proc/meminfo") as f:
    for line in f:
      k, v = line.split(":", 1)
      if k in ("Dirty", "Writeback"):
        d[k] = int(v.split()[0])
  return d.get("Dirty", 0), d.get("Writeback", 0)


def parse_kmsg(records: list[str], now_s: float, window: float = KMSG_WINDOW) -> list[str]:
  """'pri,seq,usec,flags;message' records -> '[t s] message' lines of the last window s that mention the storage stack"""
  out = []
  for rec in records:
    head, _, msg = rec.partition(";")
    parts = head.split(",")
    if len(parts) < 3 or not parts[2].isdigit():
      continue
    t = int(parts[2]) / 1e6
    msg = msg.strip()
    if now_s - t <= window and any(k in msg.lower() for k in KMSG_KEYS):
      out.append(f"[{t:.1f}] {msg[:200]}")
  return out[-KMSG_MAX:]


def read_kmsg() -> list[str]:
  """all records currently in the kernel ring buffer (non-blocking)"""
  recs = []
  fd = os.open("/dev/kmsg", os.O_RDONLY | os.O_NONBLOCK)
  try:
    while True:
      try:
        b = os.read(fd, 8192)
      except BlockingIOError:
        break
      except OSError:  # EPIPE: a record was overwritten while reading, go on
        continue
      if not b:
        break
      recs.append(b.decode(errors="replace"))
  finally:
    os.close(fd)
  return recs


def kernel_lines() -> list[str]:
  with open("/proc/uptime") as f:
    now_s = float(f.read().split()[0])
  try:
    return parse_kmsg(read_kmsg(), now_s)
  except PermissionError:
    import subprocess
    out = subprocess.run(["sudo", "-n", "dmesg", "-r"], capture_output=True, text=True, timeout=5).stdout
    recs = []
    for line in out.splitlines():  # '<pri>[  123.456789] message'
      if "]" in line and "[" in line:
        t = line[line.index("[") + 1:line.index("]")].strip()
        try:
          recs.append(f"0,0,{int(float(t) * 1e6)},-;{line[line.index(']') + 1:]}")
        except ValueError:
          pass
    return parse_kmsg(recs, now_s)


def snapshot() -> dict:
  """io counters of this user's processes (root ones cannot be read anyway; skipping them keeps a snapshot cheap)"""
  uid = os.getuid()
  procs = {}
  for pid in os.listdir("/proc"):
    if pid.isdigit():
      try:
        if os.stat(f"/proc/{pid}").st_uid != uid:
          continue
      except OSError:
        continue
      r = read_proc(pid)
      if r is not None:
        procs[pid] = r
  return {"t": time.monotonic(), "procs": procs, "disk": read_disk()}


def d_state_all() -> list[str]:
  """every process / kernel thread waiting in D state (full scan, only at a stall)"""
  out = set()
  for pid in os.listdir("/proc"):
    if pid.isdigit():
      try:
        with open(f"/proc/{pid}/stat") as f:
          st = f.read()
        if st[st.rindex(")") + 2] == "D":
          out.add(st[st.index("(") + 1:st.rindex(")")])
      except (OSError, ValueError):
        pass
  return sorted(out)


def cmd_name(pid: int, comm: str) -> str:
  """python daemons all show as 'openpilot.xxxxx' in comm: use the module name from the command line"""
  try:
    with open(f"/proc/{pid}/cmdline", "rb") as f:
      args = [a.decode(errors="replace") for a in f.read().split(b"\0") if a]
    return args[-1].rsplit(".", 1)[-1] if args and comm.startswith("openpilot.") else comm
  except OSError:
    return comm


def top_movers(old: dict, new: dict, n: int = TOP_N) -> list[tuple[str, int, float, float, float]]:
  """[(comm, pid, write MB, read MB, cancelled MB)] by bytes moved between two snapshots, largest first"""
  out = []
  for pid, (comm, _, r1, w1, c1) in new["procs"].items():
    o = old["procs"].get(pid)
    if o is None or w1 < 0 or o[3] < 0:
      continue
    dw, dr, dc = w1 - o[3], r1 - o[2], c1 - o[4]
    if dw + dr + dc > 0:
      out.append((comm, int(pid), round(dw / 1e6, 2), round(dr / 1e6, 2), round(dc / 1e6, 2)))
  out.sort(key=lambda x: -(x[2] + x[3] + x[4]))
  return [(cmd_name(pid, comm), pid, w, r, c) for comm, pid, w, r, c in out[:n]]


def record(old: dict, new: dict, iowait: float, d_state: list[str]) -> dict:
  dt = new["t"] - old["t"]
  (r0, w0, b0), (r1, w1, b1) = old["disk"], new["disk"]
  dirty, wb = read_dirty()
  return {
    "iowait": round(iowait, 3),
    "window_s": round(dt, 2),
    "top": top_movers(old, new),
    "d_state": d_state,
    "disk_w_mb": round((w1 - w0) * 512 / 1e6, 2),
    "disk_r_mb": round((r1 - r0) * 512 / 1e6, 2),
    "disk_busy_ms": b1 - b0,
    "dirty_kb": dirty,
    "writeback_kb": wb,
  }


def main():
  if not ENABLED:
    while True:
      time.sleep(3600)
  snaps = [snapshot()]
  io_prev, tot_prev = read_cpu()
  last_rec = 0.
  stall_t0 = None      # start of the current continuous stall
  last_stall = -1e9
  kmsg_done = False
  while True:
    time.sleep(SAMPLE_DT)
    try:
      io_now, tot_now = read_cpu()
      iowait = (io_now - io_prev) / max(tot_now - tot_prev, 1)
      io_prev, tot_prev = io_now, tot_now
      now = time.monotonic()
      if iowait > STALL_IOWAIT:
        stall_t0 = now if stall_t0 is None else stall_t0
        last_stall = now
      elif now - last_stall > 1.0:
        stall_t0 = None
      if now - last_stall > KMSG_REARM:
        kmsg_done = False
      if stall_t0 is not None and not kmsg_done and now - stall_t0 >= KMSG_AFTER:
        kmsg_done = True
        try:
          lines = kernel_lines()
        except Exception as e:
          lines = [f"kernel log not readable: {e}"]
        cloudlog.event("tn_iowatch: kernel log", error=True, stall_s=round(now - stall_t0, 1), lines=lines)
      if iowait > STALL_IOWAIT and now - last_rec > MIN_GAP:
        d_state = d_state_all()
        new = snapshot()
        old = next((s for s in snaps if new["t"] - s["t"] >= WINDOW), snaps[0])
        cloudlog.event("tn_iowatch: storage stall", error=True, **record(old, new, iowait, d_state))
        last_rec = now
      if now - snaps[-1]["t"] >= SNAP_DT:
        snaps.append(snapshot())
        while len(snaps) > 1 and snaps[-1]["t"] - snaps[1]["t"] >= WINDOW:
          snaps.pop(0)
    except Exception:
      cloudlog.exception("tn_iowatch: sample failed")
      time.sleep(1.0)

if __name__ == "__main__":
  main()
