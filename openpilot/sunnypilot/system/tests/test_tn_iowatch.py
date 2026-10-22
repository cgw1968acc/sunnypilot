from openpilot.sunnypilot.system import tn_iowatch as W


def snap(t, procs, disk=(0, 0, 0)):
  return {"t": t, "procs": procs, "disk": disk}


def test_top_movers_ranks_by_bytes_moved_and_skips_unreadable(monkeypatch):
  monkeypatch.setattr(W, "cmd_name", lambda pid, comm: comm)
  old = snap(0., {"1": ("loggerd", "S", 0, 0, 0), "2": ("athenad", "S", 0, 0, 0), "3": ("jbd2", "D", -1, -1, -1)})
  new = snap(2., {"1": ("loggerd", "S", 0, 8_000_000, 0), "2": ("athenad", "D", 1_000_000, 0, 3_000_000),
                  "3": ("jbd2", "D", -1, -1, -1), "4": ("new", "R", 0, 5, 0)})
  top = W.top_movers(old, new)
  assert [x[0] for x in top] == ["loggerd", "athenad"]
  assert top[1] == ("athenad", 2, 0.0, 1.0, 3.0)


def test_record_reports_disk_totals_and_d_state(monkeypatch):
  monkeypatch.setattr(W, "read_dirty", lambda: (1200, 64))
  old = snap(0., {"1": ("modeld", "S", 0, 0, 0)}, (0, 0, 0))
  new = snap(2., {"1": ("modeld", "D", 0, 0, 0)}, (2000, 40000, 250))
  r = W.record(old, new, 0.089, ["jbd2/sda12-8", "modeld"])
  assert r["d_state"] == ["jbd2/sda12-8", "modeld"]
  assert r["disk_w_mb"] == round(40000 * 512 / 1e6, 2) and r["disk_busy_ms"] == 250
  assert r["dirty_kb"] == 1200 and r["writeback_kb"] == 64
