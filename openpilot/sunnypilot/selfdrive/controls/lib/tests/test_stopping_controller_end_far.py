from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.sunnypilot.selfdrive.controls.lib.stopping_controller import StoppingController

LIM = (-3.5, 2.0)


def cs(v, standstill=False):
  return NS(vEgo=v, aEgo=-0.5, brakePressed=False, standstill=standstill, cruiseState=NS(standstill=False))


def run(sc, n, v, a_target, lead_d, lead_v, prev=-0.5):
  out = prev
  for _ in range(n):
    _, out = sc.update(LongCtrlState.pid, LongCtrlState.pid, cs(v), a_target, out, a_target, LIM, True, lead_d=lead_d, lead_v=lead_v)
  return out


def test_end_line_stays_off_while_the_lead_stop_point_is_far():
  # route 75 20:59:34.4: 9.6 km/h, plan -0.49, lead 12.2 m ahead rolling at 1.1 m/s -> the plan is sent, not the -0.65 line
  sc = StoppingController(-2.0)
  assert abs(run(sc, 50, 9.6 / 3.6, -0.49, 12.2, 1.1) - (-0.49)) < 1e-9
  assert not sc.end_active
  assert abs(run(sc, 50, 4.8 / 3.6, -0.35, 9.5, 0.0) - (-0.35)) < 1e-9       # 20:59:36: lead stopped 9.5 m ahead


def test_end_line_arms_once_the_stop_point_is_near():
  sc = StoppingController(-2.0)
  out = run(sc, 100, 8.0 / 3.6, -0.49, 5.0, 0.0)                                # 8 km/h, stopped lead 5 m: own stop 5 m -> near
  assert sc.end_active and out < -0.49                                           # the line (-0.65 at 5 km/h) is the floor


def test_end_line_arms_without_a_lead_as_before():
  sc = StoppingController(-2.0)
  out = run(sc, 100, 8.0 / 3.6, -0.49, None, None)
  assert sc.end_active and out < -0.49


def test_far_gate_uses_the_leads_run_out():
  sc = StoppingController(-2.0)
  # lead 6 m ahead but still rolling at 3 m/s (runs out 4.5 m more at 1 m/s^2): the stop point is 10.5 m away -> far
  assert abs(run(sc, 50, 8.0 / 3.6, -0.49, 6.0, 3.0) - (-0.49)) < 1e-9
  assert not sc.end_active


def test_entry_judgement_is_back_to_baseline_2():
  # owner 2026-10-22 22:09: the 94d2370 entry judgement is undone; route 9e 20:11:05.9 arms the line again as on baseline-2
  assert StoppingController.END_FAR_B_ENTRY == StoppingController.END_FAR_B == 0.3
  sc = StoppingController(-2.0)
  run(sc, 50, 12.1 / 3.6, -0.29, 11.1, 11.7 / 3.6, prev=-0.29)
  assert sc.end_active


def test_entry_judgement_leaves_the_end_of_a_stop_unchanged():
  # route 00000076: 1 km/h, stopped lead 3.6 m ahead - below END_V_HI the old judgement keeps the line (-0.36 at 1 km/h)
  sc = StoppingController(-2.0)
  run(sc, 100, 8.0 / 3.6, -0.49, 5.0, 0.0)                                       # armed on the way in
  out = run(sc, 200, 1.0 / 3.6, -0.26, 3.6, 0.0, prev=-0.36)
  assert sc.end_active and out <= -0.35
