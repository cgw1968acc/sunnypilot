import types

from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib import lead_brake_ramp as R


def lead(d, v, a, present=True):
  return types.SimpleNamespace(present=present, dRel=d, vLead=v, aLeadK=a)


def run(ctrl, n, accel, v, ld):
  out = accel
  for _ in range(n):
    out = ctrl.update(accel, v, False, ld, True, -3.5)
  return out


def test_acceleration_is_never_touched():
  # route 000000a1 20:54:28-32: the plan asked +0.6..+1.7 at 10 km/h and 0.00 was sent (min(accel, ramp=0))
  c = R.LeadBrakeRamp()
  for v in (2.0, 3.0, 5.0, 12.5, 25.0):
    for a in (0.3, 1.0, 1.7):
      assert c.update(a, v, False, lead(30.0, v + 3.0, 1.5), True, -3.5) == a       # lead pulling away
      assert c.update(a, v, False, lead(30.0, v, 0.0), True, -3.5) == a             # steady lead
      assert c.update(a, v, False, lead(0.0, 0.0, 0.0, present=False), True, -3.5) == a   # no lead
  # after a braking episode, once disarmed and let go, acceleration passes untouched again
  ld = lead(24.0, 12.5, -3.0)
  run(c, 200, -1.0, 12.5, ld)
  run(c, 300, 0.5, 12.5, lead(30.0, 15.0, 1.0))
  assert c.update(1.2, 12.5, False, lead(30.0, 15.0, 1.0), True, -3.5) == 1.2


def test_steady_follow_does_nothing():
  c = R.LeadBrakeRamp()
  assert run(c, 200, 0.0, 12.5, lead(24.0, 12.5, 0.0)) == 0.0
  assert run(c, 200, 0.0, 12.5, lead(24.0, 12.5, -0.5)) == 0.0     # a coasting lead does not arm it


def test_braking_lead_starts_soft_then_builds_steadily_to_the_need():
  c = R.LeadBrakeRamp()
  ld = lead(24.0, 12.5, -3.0)
  need = R.brake_need(12.5, 24.0, 12.5, -3.0)
  first = run(c, int(R.T_SOFT / DT_CTRL), 0.0, 12.5, ld)
  assert abs(first + R.J_SOFT * R.T_SOFT) < 0.02                    # the light touch
  prev, steps = first, []
  for _ in range(200):
    out = c.update(0.0, 12.5, False, ld, True, -3.5)
    steps.append(prev - out)
    prev = out
  assert abs(prev + need) < 1e-6                                     # reaches the need, never firmer
  assert max(steps) <= R.J_ON * DT_CTRL + 1e-9 and min(steps) >= -1e-9   # monotonic, no release on the way


def test_cap_is_off_a_firmer_plan_always_passes():
  # route 000000a3 21:08:55: the cap sent -1.81 for a -2.08 plan and the stop ended 1.2 m behind the lead
  assert R.CAP is False
  c = R.LeadBrakeRamp()
  ld = lead(24.0, 12.5, -3.0)
  run(c, 300, 0.0, 12.5, ld)
  for plan in (-1.8, -2.0, -2.5, -2.9):                                      # all firmer than the need (1.66)
    assert c.update(plan, 12.5, False, ld, True, -3.5) == plan


def test_a_firmer_plan_passes_up_to_the_cap_and_a_lighter_one_is_firmed(monkeypatch):
  monkeypatch.setattr(R, "CAP", True)
  c = R.LeadBrakeRamp()
  ld = lead(24.0, 12.5, -3.0)
  need = R.brake_need(12.5, 24.0, 12.5, -3.0)
  run(c, 300, 0.0, 12.5, ld)
  assert abs(c.update(-0.5, 12.5, False, ld, True, -3.5) + need) < 1e-6
  cap = need * R.CAP_K + R.CAP_ADD
  assert abs(c.update(-2.9, 12.5, False, ld, True, -3.5) + cap) < 1e-6
  assert c.update(-3.2, 12.5, False, ld, True, -3.5) == -3.2                # an emergency is never held back


def test_disarming_opens_the_cap_gradually(monkeypatch):
  monkeypatch.setattr(R, "CAP", True)
  # route 0000009e 20:11:04: the cap vanished at the disarm and the request stepped 0.15 firmer
  c = R.LeadBrakeRamp()
  ld = lead(24.0, 12.5, -3.0)
  run(c, 300, -2.6, 12.5, ld)
  prev = c.update(-2.6, 12.5, False, ld, True, -3.5)
  assert prev > -2.6                                                        # capped
  tap_over = lead(24.0, 12.0, 0.0)                                          # the lead stops braking: disarm
  worst = 0.0
  for _ in range(300):
    out = c.update(-2.6, 12.5, False, tap_over, True, -3.5)
    worst = max(worst, prev - out)
    prev = out
  assert not c.armed and abs(prev + 2.6) < 1e-6                             # back to the plan ...
  assert worst <= R.J_REL * DT_CTRL + 1e-9                                  # ... at J_REL, no step


def test_arming_meets_the_request_without_a_step():
  c = R.LeadBrakeRamp()
  quiet = lead(24.0, 12.5, 0.0)
  run(c, 50, -2.0, 12.5, quiet)
  out = c.update(-2.0, 12.5, False, lead(24.0, 12.5, -3.0), True, -3.5)
  assert abs(out + 2.0) < 1e-6


def test_brake_tap_lets_go():
  c = R.LeadBrakeRamp()
  run(c, 50, 0.0, 12.5, lead(24.0, 12.5, -1.2))
  assert c.ramp < 0.0
  out = run(c, int(1.0 / DT_CTRL), 0.0, 12.5, lead(24.0, 12.0, -0.1))   # the lead stops braking
  assert out == 0.0 and not c.armed


def test_stopped_lead_keeps_it_and_low_speed_hands_back():
  c = R.LeadBrakeRamp()
  run(c, 300, 0.0, 8.0, lead(20.0, 4.0, -3.0))
  assert c.armed
  assert run(c, 5, 0.0, 6.0, lead(12.0, 0.0, 0.0)) < 0.0             # lead stood, aLeadK back to 0: still armed
  assert c.armed
  assert c.update(-0.2, R.V_FADE[0] - 0.1, False, lead(5.0, 0.0, 0.0), True, -3.5) == -0.2   # below the fade: untouched


def test_off_paths():
  c = R.LeadBrakeRamp()
  ld = lead(24.0, 12.5, -3.0)
  run(c, 100, 0.0, 12.5, ld)
  assert c.update(-0.1, 12.5, False, ld, True, -3.5, hold_off=True) == -0.1
  assert c.update(-0.1, 12.5, False, ld, False, -3.5) == -0.1 and c.ramp == 0.0
  try:
    R.ENABLED = False
    assert run(R.LeadBrakeRamp(), 100, 0.0, 12.5, ld) == 0.0
  finally:
    R.ENABLED = True
