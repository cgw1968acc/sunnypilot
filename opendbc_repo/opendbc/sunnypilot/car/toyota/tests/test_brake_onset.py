import numpy as np

from opendbc.car import rate_limit, DT_CTRL
from opendbc.sunnypilot.car.toyota.brake_onset import (BrakeOnsetShaper, EngageOnsetShaper, ENGAGE_T_BP, ENGAGE_J_UP, ONSET_J_DOWN,
                                                       HARD_BRAKE_ACCEL, URGENT_T, URGENT_J)

DT = DT_CTRL * 3
STOCK_J = 4.0
UP_STEP = 4.0 * DT


def run(requests, a0=0.0, fcw=False):
  shaper = BrakeOnsetShaper(DT, STOCK_J)
  a = a0
  out = []
  for req in requests:
    step = shaper.down_step(req, a, bypass=fcw, urgent=shaper.is_urgent(req, False))
    a = rate_limit(req, a, step, UP_STEP)
    out.append(a)
  return np.array(out)


def at(out, t):
  return out[int(round(t / DT)) - 1]


class TestBrakeOnset:
  def test_step_brake_eases_in_only_for_the_first_instant(self):
    # 2026-10-01 shape: 1.0 m/s^3 for the first 0.1 s (light touch), then a linear build to the stock 4.0 m/s^3 by 0.3 s
    out = run([-1.5] * 60)
    assert at(out, 0.09) >= -0.12            # first 0.1 s: a light touch
    assert -0.45 < at(out, 0.21) < -0.12     # the linear build is under way
    assert at(out, 0.51) < -0.9              # most of the way by 0.5 s
    assert np.isclose(at(out, 0.81), -1.5, atol=1e-6)  # the full request by ~0.8 s

  def test_jerk_follows_the_schedule_and_never_exceeds_stock(self):
    out = run([-1.9] * 60, a0=1.0)  # large step, still above the hard-brake bypass
    jerks = np.diff(np.concatenate([[1.0], out])) / DT
    assert jerks.min() >= -STOCK_J - 1e-6
    assert jerks[0] >= -ONSET_J_DOWN[0] - 1e-6
    assert jerks[int(0.09 / DT)] >= -ONSET_J_DOWN[1] - 1e-6
    # the blend is one straight ramp: jerk keeps growing monotonically until the stock limit
    ramp = -jerks[int(0.1 / DT):int(0.6 / DT)]
    assert np.all(np.diff(ramp) >= -1e-6)

  def test_lead_switch_during_a_settled_brake_gets_the_same_soft_onset(self):
    reqs = [-0.6] * 70 + [-1.8] * 60  # long enough for the schedule to wind fully back before the switch
    out = run(reqs)
    t_switch = 70 * DT
    assert np.isclose(at(out, t_switch), -0.6, atol=1e-6)
    assert at(out, t_switch + 0.09) >= -0.6 - 0.12
    assert np.isclose(out[-1], -1.8, atol=1e-6)

  def test_hard_brake_request_gets_a_short_soft_start_then_stock(self):
    out = run([HARD_BRAKE_ACCEL - 1.0] * 30)
    assert np.isclose(out[0], -URGENT_J * DT, atol=1e-6)          # first frame: 1.0 m/s^3, not the stock 4.0
    assert at(out, URGENT_T) >= -URGENT_J * URGENT_T - 0.03        # ~-0.1 after the light 0.1 s
    assert at(out, 0.36) <= -0.8                                   # then a linear build at the stock rate
    jerks = np.diff(np.concatenate([[0.0], out])) / DT
    assert jerks.min() >= -STOCK_J - 1e-6

  def test_fcw_bypasses_the_schedule(self):
    out = run([-1.5] * 20, fcw=True)
    assert np.isclose(out[0], -STOCK_J * DT, atol=1e-6)

  def test_release_is_untouched(self):
    out = run([1.0] * 20, a0=-1.5)
    assert np.isclose(out[0], -1.5 + UP_STEP, atol=1e-6)

  def test_gentle_request_passes_straight_through(self):
    reqs = list(np.linspace(0.0, -0.2, 40))  # ~0.17 m/s^3, below the gentlest onset limit (0.25)
    out = run(reqs)
    assert np.allclose(out, reqs, atol=1e-9)

  def test_short_pause_does_not_fully_rearm(self):
    shaper = BrakeOnsetShaper(DT, STOCK_J)
    a = 0.0
    for _ in range(int(0.3 / DT)):
      a = rate_limit(-1.5, a, shaper.down_step(-1.5, a), UP_STEP)
    t_after_ramp = shaper.t_onset
    for _ in range(2):  # hold for two frames, request steady
      shaper.down_step(a, a)
    assert 0.0 < shaper.t_onset < t_after_ramp
    for _ in range(int(1.0 / DT)):  # a full second of settled brake winds it back completely
      shaper.down_step(a, a)
    assert shaper.t_onset == 0.0


class TestEngageOnsetShaper:
  DT = 0.03
  STOCK = 4.0

  def _sim(self, request=1.0, frames=40):
    sh = EngageOnsetShaper(self.DT, self.STOCK)
    cmd, out = 0.0, []
    for _ in range(frames):
      step = sh.up_step(True)
      cmd = min(request, cmd + step)
      out.append(cmd)
    return out

  def test_first_instant_is_almost_nothing_then_blends_to_stock(self):
    out = self._sim()
    t = [self.DT * (i + 1) for i in range(len(out))]

    def at(s):
      return out[max(i for i, ti in enumerate(t) if ti <= s + 1e-9)]
    assert at(0.09) < 0.04            # ~0.25-0.5 m/s^3 for the first 0.1 s: a few hundredths of a m/s^2
    assert 0.05 < at(0.3) < 0.35      # building
    assert at(0.9) >= 0.99            # joined the stock ramp, request reached well under a second
    jerks = [(out[i] - out[i - 1]) / self.DT for i in range(1, len(out))]
    assert max(jerks) <= self.STOCK + 1e-9

  def test_schedule_is_monotonic_and_ends_at_stock(self):
    assert list(ENGAGE_J_UP) == sorted(ENGAGE_J_UP)
    assert ENGAGE_J_UP[-1] == self.STOCK
    assert ENGAGE_T_BP[0] == 0.0

  def test_unshaped_once_settled_and_rearms_on_disengage(self):
    sh = EngageOnsetShaper(self.DT, self.STOCK)
    for _ in range(int(1.0 / self.DT)):
      sh.up_step(True)
    assert sh.up_step(True) == self.STOCK * self.DT
    assert sh.up_step(False) == self.STOCK * self.DT
    assert sh.up_step(True) < 0.3 * self.DT     # re-armed: engage schedule from the start again

  def test_engage_window_covers_the_first_0_6_s_only(self):
    sh = EngageOnsetShaper(self.DT, self.STOCK)
    assert sh.in_engage_window                     # not yet ticked: the engage instant
    for _ in range(int(0.5 / self.DT)):
      sh.up_step(True)
    assert sh.in_engage_window
    for _ in range(int(0.3 / self.DT)):
      sh.up_step(True)
    assert not sh.in_engage_window
    sh.up_step(False)
    assert sh.in_engage_window                     # re-armed by a disengage


class TestLowSpeedOnset:
  DT = 0.03
  STOCK = 4.0

  def _sim(self, v_ego, request=-1.5, frames=60):
    sh = BrakeOnsetShaper(self.DT, self.STOCK)
    cmd, out = 0.0, []
    for _ in range(frames):
      cmd = max(request, cmd + sh.down_step(request, cmd, v_ego=v_ego))
      out.append(cmd)
    return out

  def test_same_shape_at_every_speed(self):
    # the old slower low-speed join delayed stops so much that the governor had to catch up (device campaign 2026-10-01)
    slow = self._sim(30 / 3.6)
    fast = self._sim(80 / 3.6)
    assert np.allclose(slow, fast)
    assert slow[int(0.9 / self.DT)] <= -1.5 + 1e-9

  def test_first_instant_is_the_same_lightest_touch_at_any_speed(self):
    for v in (20 / 3.6, 50 / 3.6, 100 / 3.6):
      out = self._sim(v)
      assert out[int(0.09 / self.DT)] >= -0.12


def test_handover_scale_flattens_only_the_3_to_5_kph_bite():
  from opendbc.sunnypilot.car.toyota.brake_onset import handover_scale
  assert handover_scale(10 / 3.6, -0.7) == -0.7          # untouched above the handover
  assert handover_scale(1.5 / 3.6, -0.5) == -0.5         # untouched in the final crawl (-0.026 lock lives below 1 km/h)
  assert -0.6 < handover_scale(4.5 / 3.6, -0.7) < -0.5   # ~0.78-0.84 in the 3.8-4.8 km/h band
  assert handover_scale(4.5 / 3.6, 0.3) == 0.3           # throttle never scaled
