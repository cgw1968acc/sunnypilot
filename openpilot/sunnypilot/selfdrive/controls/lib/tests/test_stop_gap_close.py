from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap_close import StopGapClose
from openpilot.sunnypilot.selfdrive.controls.lib.end_force_limit import EndForceLimit


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
  for v0, d, plain in ((11.2, 16.5, -0.57), (12.0, 12.1, -0.88), (6.0, 9.0, -0.7), (4.1, 7.5, -0.65)):
    for tau in (0.35, 0.6, 0.9):
      for creep in (0.20, 0.25, 0.30):
        gap, hist = sim(v0 / 3.6, d, plain=plain, creep=creep, tau=tau)
        assert 3.0 <= gap <= 4.4, (v0, tau, creep, gap)
        assert len(hist) < 1200, (v0, tau, creep)  # it does stop (no endless crawl)
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
