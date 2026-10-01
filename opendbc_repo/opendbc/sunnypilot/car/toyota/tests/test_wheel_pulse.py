from opendbc.sunnypilot.car.toyota.wheel_pulse import WheelPulseCreep, CREEP_HOLD_T, V_FLOOR, DT


def run(seq):
  w = WheelPulseCreep()
  return [w.update(enc, zero) for enc, zero in seq]


def test_normal_stop_no_forward_step_after_zero():
  # steps while moving, then the wheels read 0 and only a rebound (-1) follows: never creeping
  seq = [(float(4 * (i // 25)), False) for i in range(200)]
  last = seq[-1][0]
  seq += [(last, True)] * 50 + [(last - 1, True)] * 300
  assert max(run(seq)[200:]) == 0.0


def test_forward_step_below_the_floor_is_creep():
  seq = [(float(4 * (i // 25)), False) for i in range(200)]
  last = seq[-1][0]
  seq += [(last, True)] * 150 + [(last + 4, True)] * 400       # a step 1.5 s after the wheels read 0
  out = run(seq)
  k = 200 + 150
  assert 0.0 < out[k] <= V_FLOOR
  assert all(o > 0.0 for o in out[k:k + int(CREEP_HOLD_T / DT) - 1])
  assert out[k + int(CREEP_HOLD_T / DT) + 2] == 0.0
  assert all(b <= a + 1e-12 for a, b in zip(out[k:k + 240], out[k + 1:k + 241]))   # only decays between steps


def test_no_encoder_no_change():
  assert max(run([(float('nan'), True)] * 300)) == 0.0


def test_wrap_around():
  seq = [(252.0, False)] * 30 + [(252.0, True)] * 100 + [(0.0, True)] * 10
  assert run(seq)[-1] > 0.0
