import pytest

from openpilot.cereal import log
import openpilot.sunnypilot.selfdrive.controls.lib.stop_gap_close as stop_gap_close
from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap_close import StopGapClose
from openpilot.sunnypilot.selfdrive.controls.lib.end_force_limit import EndForceLimit


@pytest.fixture(autouse=True)
def gap_close_on(monkeypatch):
  # tncr ships it OFF (ENABLED = False); the behaviour itself is tested switched on
  monkeypatch.setattr(stop_gap_close, "ENABLED", True)


def lead(d, v=0.0, present=True):
  l = log.RadarState.LeadData.new_message()
  l.present = present
  l.vLead = v
  l.dRel = d
  return l


def run(c, n, accel, v, a_ego, l, standstill=False, active=True):
  out = accel
  for _ in range(n):
    out = c.update(accel, v, a_ego, standstill, l, active)
  return out


def sim(v, d, plain=-0.6, creep=0.27, tau=0.6, delay=0.2, t_max=30.0):
  """car: aEgo follows the command (+ creep torque below 5 km/h) after a transport delay and a first-order lag;
  stopped lead d ahead; the chain is stop_gap_close -> end_force_limit as in controlsd"""
  c, e = StopGapClose(), EndForceLimit()
  run(c, 60, plain, v, plain, lead(d))  # braking for 0.6 s
  a, x, hist, buf = plain, 0.0, [], [plain] * int(delay / 0.01)
  for _ in range(int(t_max / 0.01)):
    cmd = e.update(c.update(plain, v, a, False, lead(d - x), True), v, False, True, a)
    hist.append((v, cmd, c.active))
    buf.append(cmd)
    u = buf.pop(0)
    # below 5 km/h the hybrid's creep torque: BRAKE_FORCE ~ 480 + 1200 * |cmd| N against ~750 N of creep on ~1000 kg
    # effective -> decel ~ 1.2 * |cmd| - creep (route 00000159: -0.55 -> -0.44, -0.30 -> ~0)
    a_tgt = (1.2 * u + creep) if (v < 5 / 3.6 and u < 0.0) else u
    a += (a_tgt - a) * 0.01 / tau
    v = max(v + a * 0.01, 0.0)
    x += v * 0.01
    if v == 0.0:
      break
  return d - x, hist


def test_passes_through_when_the_stop_is_not_far():
  assert run(StopGapClose(), 200, -0.8, 8 / 3.6, -0.8, lead(5.0)) == -0.8           # 5 - 3.1 = 1.9 m
  assert run(StopGapClose(), 200, -0.5, 8 / 3.6, -0.5, lead(15.0, v=2.0)) == -0.5   # moving lead
  assert run(StopGapClose(), 200, -0.5, 20 / 3.6, -0.5, lead(30.0)) == -0.5         # above 12 km/h
  assert run(StopGapClose(), 200, -2.5, 8 / 3.6, -2.5, lead(12.0)) == -2.5          # hard brake


def test_a_mid_stop_decel_does_not_predict_far():
  c = StopGapClose()  # route 00000159 09:14:41: -1.14 at 12 km/h, 9.3 m; with the 0.9 cap it predicts 3.1 m
  run(c, 100, -1.14, 12 / 3.6, -1.4, lead(9.3))
  assert not c.active


def test_far_stop_rolls_on_steadily_into_the_window():
  # 19:35:35 on route 0000015c (stopped 5.5 m out) and others; robust to PCM lag and creep torque spread
  for v0, d, plain in ((11.2, 16.5, -0.57), (12.0, 12.1, -0.88), (10.1, 8.9, -1.05), (11.0, 16.0, -0.5),
                       (6.0, 9.0, -0.7), (4.1, 7.5, -0.65)):
    for tau in (0.35, 0.6, 0.9):
      for creep in (0.20, 0.25, 0.30):
        gap, hist = sim(v0 / 3.6, d, plain=plain, creep=creep, tau=tau)
        if v0 > 8.0:
          assert 3.0 <= gap <= 4.5, (v0, tau, creep, gap)  # tncr: held while it ends 3.35-3.8 m; the slowest plant overshoots a bit
        else:  # tncr: a late trigger eases at only 0.3 m/s^3 - it closes less, but never ends farther than without it
          stop_gap_close.ENABLED = False
          gap_off, _ = sim(v0 / 3.6, d, plain=plain, creep=creep, tau=tau)
          stop_gap_close.ENABLED = True
          assert 3.0 <= gap <= gap_off + 0.05, (v0, tau, creep, gap, gap_off)
        assert len(hist) < (1200 if v0 > 8.0 else 1500), (v0, tau, creep)  # it does stop (no endless crawl; tncr's slow
        # ease makes a 6 km/h late trigger with the strongest creep torque take up to ~13 s)
        lo, rise = 99.0, 0.0
        for v, _, _ in hist:  # fast-slow-fast: how much the speed rises again after a dip
          lo = min(lo, v)
          rise = max(rise, v - lo)
        assert rise < 0.15 / 3.6, (v0, tau, creep, rise * 3.6)


def test_end_is_minus_035_at_zero():  # tncr (Corolla Cross): -0.35; Altis -0.30
  for creep in (0.20, 0.30):
    _, hist = sim(6 / 3.6, 9.0, plain=-0.7, creep=creep)
    assert -0.355 <= hist[-1][1] <= -0.33
    below = [cmd for v, cmd, _ in hist if v < 2.0 / 3.6]
    assert min(below) >= -0.8


def test_lead_moves_or_drops_gives_the_request_back():
  c = StopGapClose()
  run(c, 60, -0.7, 6 / 3.6, -0.7, lead(9.0))
  assert c.active
  run(c, 1, -0.7, 3 / 3.6, -0.3, lead(8.0, v=1.5))   # lead drives off
  assert not c.active
  assert run(c, 200, 0.3, 3 / 3.6, 0.0, lead(8.0, v=1.5)) == 0.3
  c = StopGapClose()
  run(c, 60, -0.7, 6 / 3.6, -0.7, lead(9.0))
  run(c, 40, -0.7, 3 / 3.6, -0.3, lead(9.0, present=False))  # radar lead lost 0.4 s
  assert not c.active
  c = StopGapClose()
  run(c, 60, -0.7, 6 / 3.6, -0.7, lead(9.0))
  run(c, 1, -0.7, 2 / 3.6, -0.3, lead(8.5))
  run(c, 1, -0.7, 2 / 3.6, -0.3, lead(2.9 + 5.0))
  assert c.active
  run(c, 1, -0.7, 2 / 3.6, -0.3, lead(7.0))  # 0.9 m step: same lead
  assert c.active


def test_reset_at_standstill_and_disengage():
  c = StopGapClose()
  run(c, 60, -0.7, 6 / 3.6, -0.7, lead(9.0))
  assert run(c, 1, -0.4, 0.0, 0.0, lead(3.3), standstill=True) == -0.4
  assert not c.active and c.out is None
  run(c, 60, -0.7, 6 / 3.6, -0.7, lead(9.0))
  assert run(c, 1, -0.7, 6 / 3.6, -0.7, lead(9.0), active=False) == -0.7 and not c.active


def test_off_passes_the_request_through(monkeypatch):
  monkeypatch.setattr(stop_gap_close, "ENABLED", False)
  c = StopGapClose()
  for _ in range(200):  # route 00000069 00:40:20.7: -1.05 at 10 km/h, 8.9 m behind a stopped lead
    assert c.update(-1.05, 10.0 / 3.6, -1.1, False, lead(8.9), True) == -1.05
  assert not c.active


def test_tncr_eases_slowly_and_never_coasts():
  # route 00000069 00:40:20.7: -1.05 at 10 km/h, 8.9 m. f908530 eased -1.06 -> -0.58 in 0.4 s; now at most 0.3 m/s^3
  for tau in (0.35, 0.6, 0.9):
    _, hist = sim(10.1 / 3.6, 8.9, plain=-1.05, tau=tau)
    above = [(v, cmd) for v, cmd, _ in hist if v > 2.0 / 3.6]
    for (_, c0), (_, c1) in zip(above, above[1:], strict=False):
      assert c1 - c0 <= stop_gap_close.RELEASE_RATE * 0.01 + 1e-9
    assert min(cmd for _, cmd in above) <= -0.3 and max(cmd for _, cmd in above) <= -0.3 + 1e-9  # no coast above 2 km/h


def test_tncr_feedback_only_firms_up():
  # decelerating harder than needed must not lighten the request (that pumped on route 0000006a 00:48:24)
  c1, c2 = StopGapClose(), StopGapClose()
  run(c1, 60, -0.6, 11 / 3.6, -0.6, lead(16.0))
  run(c2, 60, -0.6, 11 / 3.6, -0.6, lead(16.0))
  out_hard = run(c1, 100, -0.6, 11 / 3.6, -1.2, lead(16.0))   # car decelerates much more than b
  out_plain = run(c2, 100, -0.6, 11 / 3.6, -0.45, lead(16.0)) # about b
  assert c1.active and c2.active
  assert abs(out_hard - out_plain) < 0.02


def test_tncr_far_gap_holds_the_present_force():
  # owner 2026-10-15: too far -> hold the present force longer, neither release nor add brake. Route 0000006a 00:48:25:
  # 11 km/h, 15.5 m, -0.48 asked: holding -0.40 ends ~3.6 m -> the request stays put while the car slows as asked
  c = StopGapClose()
  run(c, 60, -0.45, 11.4 / 3.6, -0.45, lead(16.2))
  assert c.active
  outs = []
  v, d = 11.1 / 3.6, 15.5
  for _ in range(100):
    b_hold = 0.40
    outs.append(c.update(-0.50, v, -b_hold, False, lead(d), True))
    d -= v * 0.01
    v -= b_hold * 0.01
  steady = outs[50:]
  assert max(steady) - min(steady) < 0.02
