from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap_hold import StopGapHold


def lead(d, v=0.0, present=True):
  l = log.RadarState.LeadData.new_message()
  l.present = present
  l.vLead = v
  l.dRel = d
  return l


def run(h, n, accel, v, a_ego, l, standstill=False):
  out = accel
  for _ in range(n):
    out = h.update(accel, v, a_ego, standstill, l, True)
  return out


V12 = 12 / 3.6  # 3.33 m/s: at -1.2 the stop takes 4.6 m


def test_passes_through_when_the_stop_is_safe():
  h = StopGapHold()
  assert run(h, 200, -1.5, V12, -1.5, lead(15.0)) == -1.5
  assert run(h, 10, -1.0, V12, -1.5, lead(15.0)) == -1.0     # easing with room to spare: untouched
  assert run(StopGapHold(), 200, -1.0, V12, -1.0, lead(5.0, v=3.0)) == -1.0  # moving lead: not this rule
  assert run(StopGapHold(), 200, -1.0, 35 / 3.6, -1.0, lead(5.0)) == -1.0    # above 30 km/h


def test_holds_instead_of_easing_when_too_close():
  h = StopGapHold()
  run(h, 150, -1.5, V12, -1.2, lead(7.0))                  # 7 - 3.33^2 / 2.4 = 2.4 m < 3.5
  assert h.holding
  assert run(h, 10, -1.2, V12, -1.2, lead(7.0)) == -1.5    # the plan eases: held
  assert run(h, 10, -1.8, V12, -1.2, lead(7.0)) == -1.8    # firmer is always allowed
  assert run(h, 10, -1.2, V12, -1.2, lead(7.0)) == -1.8


def test_not_at_the_first_touch():
  h = StopGapHold()
  assert run(h, 50, -0.9, 9 / 3.6, 0.0, lead(9.0)) == -0.9  # just engaged, still within the first second
  assert run(h, 10, -0.6, 9 / 3.6, 0.0, lead(9.0)) == -0.6  # the first touch may ease


def test_linear_release_once_safe_again():
  h = StopGapHold()
  run(h, 150, -1.5, V12, -1.2, lead(7.0))
  out = run(h, 20, -1.0, V12, -1.2, lead(20.0))            # safe again: releases at 1.0 m/s^3, 0.2 s -> 0.2
  assert abs(out - (-1.3)) < 1e-6
  assert run(h, 100, -1.0, V12, -1.2, lead(20.0)) == -1.0  # back on the plain request


def test_2_to_0_end_untouched():
  h = StopGapHold()
  run(h, 150, -1.5, V12, -1.2, lead(5.0))
  assert abs(run(h, 1, -0.8, 4.5 / 3.6, -1.0, lead(2.0)) - (-1.15)) < 1e-6  # 4.5 km/h: half way back to the request
  assert run(h, 1, -0.5, 2.0 / 3.6, -0.6, lead(1.5)) == -0.5                # 2 km/h and below: the end line as it is
  assert run(h, 1, -0.4, 0.0, 0.0, lead(1.5), standstill=True) == -0.4
  assert h.held is None
