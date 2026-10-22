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


def test_a_firmer_plan_passes_up_to_the_cap_and_a_lighter_one_is_firmed():
  c = R.LeadBrakeRamp()
  ld = lead(24.0, 12.5, -3.0)
  need = R.brake_need(12.5, 24.0, 12.5, -3.0)
  run(c, 300, 0.0, 12.5, ld)
  assert abs(c.update(-0.5, 12.5, False, ld, True, -3.5) + need) < 1e-6
  cap = need * R.CAP_K + R.CAP_ADD
  assert abs(c.update(-3.3, 12.5, False, ld, True, -3.5) + cap) < 1e-6
  assert c.update(-1.0 - cap, 12.5, False, ld, True, -3.5) >= -cap - 1e-6


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
