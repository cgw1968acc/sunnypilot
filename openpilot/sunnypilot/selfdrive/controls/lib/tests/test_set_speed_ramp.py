import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.set_speed_ramp import SetSpeedRamp

DT = 0.05


def run(v0, step, a_max, a0=0.0, n=1000):
  r = SetSpeedRamp()
  v = v0
  vc = v0 + step
  r.start(v, vc, a0, a_max)
  out = []
  for _ in range(n):
    a = r.update(DT, v, vc)
    if a is None:
      break
    out.append(a)
    v += a * DT
  return np.array(out), v


def test_plus5_at_31kph_is_one_smooth_bell():
  a, v_end = run(31 / 3.6, 5 / 3.6, a_max=0.9)
  assert 0.65 < a.max() < 0.72                            # was ~1.0 within 1 s
  assert abs(v_end - 36 / 3.6) < 0.1                      # reaches the new speed
  h = len(a) // 2
  assert np.all(np.diff(a[:h - 1]) >= -1e-9) and np.all(np.diff(a[h + 1:]) <= 1e-9)
  assert np.max(np.abs(np.diff(a))) / DT < 0.6            # never a hard jerk


def test_soft_first_touch_and_soft_arrival():
  a, _ = run(31 / 3.6, 5 / 3.6, a_max=0.9)
  assert a[0] < 0.01 and a[-1] < 0.01


def test_large_step_holds_the_profile_max():
  a, _ = run(60 / 3.6, 20 / 3.6, a_max=0.3)
  assert abs(a.max() - 0.3) < 1e-9 and np.sum(np.isclose(a, 0.3)) > 20


def test_repeated_press_continues_from_current_accel():
  r = SetSpeedRamp()
  r.start(10.0, 11.4, 0.3, 0.9)
  assert abs(r.update(DT, 10.0, 11.4) - 0.3) < 0.03


def test_no_ramp_when_already_faster_or_tiny_step():
  r = SetSpeedRamp()
  r.start(10.0, 11.4, 0.9, 0.9)
  assert r.update(DT, 10.0, 11.4) is None
  r.start(10.0, 10.1, 0.0, 0.9)
  assert r.update(DT, 10.0, 10.1) is None
