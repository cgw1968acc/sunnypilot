import numpy as np
from openpilot.sunnypilot.selfdrive.controls.lib.end_force_limit import EndForceLimit, V_LINE, END_ACCEL, CEIL_BP, CEIL_A


def ceil(kph):
  return float(np.interp(kph / 3.6, CEIL_BP, CEIL_A))


def test_line_from_2kph_to_minus_030():
  e = EndForceLimit()
  assert e.update(-0.92, 2.8 / 3.6, False, True) == ceil(2.8)    # above the 2 km/h line: the ceiling (-0.37, owner 10-22)
  assert e.update(-0.80, 2.0 / 3.6, False, True) == ceil(2.0)    # tncr: the ceiling (-0.60) caps it; a_line = -0.60
  out = e.update(-0.70, 1.0 / 3.6, False, True)                  # 18:51: -0.70 at 1.4 km/h asked -> ceiling -0.38
  assert abs(out - max(ceil(1.0), END_ACCEL + (ceil(2.0) - END_ACCEL) * 0.5)) < 1e-9
  assert e.update(-0.46, 0.01, False, True) >= END_ACCEL - 0.01 * 0.5 / V_LINE - 1e-9  # at the stop: ~-0.34
  assert e.update(-0.46, 0.0, False, True) == END_ACCEL


def test_lighter_passes_and_never_firmer_than_030_at_zero():
  e = EndForceLimit()
  e.update(-0.25, 2.5 / 3.6, False, True)
  assert e.update(-0.25, 1.0 / 3.6, False, True) == -0.25
  assert e.update(-2.5, 0.0, False, True) == END_ACCEL            # no exceptions


def test_standstill_continues_into_the_hold_without_a_step():
  e = EndForceLimit()
  e.update(-0.6, 2.0 / 3.6, False, True)
  assert e.update(-0.6, 0.0, False, True) == END_ACCEL
  out = e.update(-1.2, 0.0, True, True)                           # the hold asks -1.2 at once
  assert abs(out - (END_ACCEL - 0.005)) < 1e-9                    # firms up at 0.5 m/s^3
  for _ in range(300):
    out = e.update(-1.2, 0.0, True, True)
  assert out == -1.2 and e.out is None
  assert e.update(0.5, 0.0, True, True) == 0.5                    # launch passes


def test_engaged_below_the_line():
  e = EndForceLimit()
  ceil15 = ceil(1.5)                                                      # tncr: the fixed ceiling caps -0.9 at 1.5 km/h
  assert abs(e.update(-0.9, 1.5 / 3.6, False, True) - ceil15) < 1e-9     # and the line is anchored there
  out = e.update(-0.9, 0.75 / 3.6, False, True)
  assert abs(out - max(ceil(0.75), END_ACCEL + (ceil15 - END_ACCEL) * 0.5)) < 1e-9
  assert e.update(-0.9, 0.0, False, True) == END_ACCEL


def test_stall_lets_the_line_go_and_reanchors():
  e = EndForceLimit()
  e.update(-0.6, 2.0 / 3.6, False, True, -0.5)
  assert e.update(-0.8, 1.5 / 3.6, False, True, -0.5) > -0.8      # on the line
  for _ in range(30):                                              # rolling at 1.5 km/h without slowing for 0.3 s
    out = e.update(-0.8, 1.5 / 3.6, False, True, 0.0)
  assert out == ceil(1.5)                                          # a stall is not the stop: the request passes - up to the
  e.update(-0.8, 1.4 / 3.6, False, True, -0.4)                     # ceiling (tncr: -0.49 at 1.5 km/h); then re-anchored there
  out = e.update(-0.8, 0.7 / 3.6, False, True, -0.4)
  assert abs(out - max(ceil(0.7), END_ACCEL + (ceil(1.5) - END_ACCEL) * 0.7 / 1.5)) < 1e-6
  assert e.update(-0.8, 0.0, False, True, -0.4) == END_ACCEL


def test_fixed_ceiling_from_5kph_caps_everything():  # tncr 2026-10-15 evening: -2.0 @5 -> -0.34 @0, no exceptions
  e = EndForceLimit()
  assert e.update(-2.7, 6.0 / 3.6, False, True) == -2.7                       # above 5 km/h: untouched
  # owner: 5 km/h may still need a firm brake; the last 1-0 km/h must be below -0.6, best -0.34
  # 2026-10-22 owner: 3 -0.37, 2 -0.37, 1 -0.36, 0 -0.34 (below 2 km/h the line from the 2 km/h anchor: 1 km/h -0.355)
  for kph, want in ((5.0, -2.0), (4.0, -1.185), (3.0, -0.37), (2.0, -0.37), (1.4, -0.361), (1.0, -0.355), (0.5, -0.3475),
                    (0.0, END_ACCEL)):
    out = e.update(-2.7, kph / 3.6, False, True)
    assert abs(out - want) < 0.005, (kph, out)
  e2 = EndForceLimit()
  assert e2.update(-0.65, 5.0 / 3.6, False, True) == -0.65                    # an ordinary end (END curve) never touches it
  assert e2.update(-0.60, 3.5 / 3.6, False, True) == -0.60                    # -0.78 ceiling at 3.5 km/h
  assert e2.update(-0.60, 3.0 / 3.6, False, True) == -0.37                    # owner line from 3 km/h
  e3 = EndForceLimit()                                                        # into the standstill hold without a step
  e3.update(-2.7, 1.0 / 3.6, False, True)
  assert e3.update(-2.7, 0.0, False, True) == END_ACCEL
  out = e3.update(-1.2, 0.0, True, True)
  assert abs(out - (END_ACCEL - 0.005)) < 1e-9


def _roll(lim, n, req, v, a_ego, lead_d):
  out = None
  for _ in range(n):
    out = lim.update(req, v, False, True, a_ego, lead_d=lead_d)
  return out


def test_short_gap_exception_route_a3():
  # route 000000a3 21:09:00: 2.2 km/h, lead 2.2 m, aEgo -0.1 (creeping), plan -0.69 -> the ceiling (-0.37) gives way
  from openpilot.sunnypilot.selfdrive.controls.lib import end_force_limit as E
  lim = E.EndForceLimit()
  first = lim.update(-0.69, 2.2 / 3.6, False, True, -0.1, lead_d=2.2)
  assert -0.37 - E.EXC_RATE * 0.01 - 1e-9 <= first < -0.37                    # starts at the ceiling, no bite
  out = _roll(lim, 50, -0.69, 2.2 / 3.6, -0.1, 2.2)
  assert abs(out + 0.69) < 1e-9                                                # reaches the request, not firmer
  assert _roll(lim, 50, -2.5, 2.2 / 3.6, -0.1, 2.0) >= E.EXC_MAX - 1e-9         # bounded


def test_short_gap_exception_never_opens_on_an_ordinary_stop():
  # route 000000a3 21:05:24: 1.6 km/h, lead 3.7 m, aEgo -0.45 -> predicted stop 3.5 m: the ceiling holds
  from openpilot.sunnypilot.selfdrive.controls.lib import end_force_limit as E
  lim = E.EndForceLimit()
  assert _roll(lim, 30, -0.46, 1.6 / 3.6, -0.45, 3.7) >= -0.37
  assert lim.exc is None
  assert _roll(E.EndForceLimit(), 30, -0.9, 3.0 / 3.6, -0.9, 3.2) >= -0.37      # 3 km/h, 3.2 m, braking well
  assert _roll(E.EndForceLimit(), 30, -0.9, 2.0 / 3.6, -0.1, None) >= -0.37     # no lead: no exception


def test_short_gap_exception_eases_with_the_request():
  from openpilot.sunnypilot.selfdrive.controls.lib import end_force_limit as E
  lim = E.EndForceLimit()
  _roll(lim, 60, -0.9, 2.0 / 3.6, -0.1, 2.0)
  prev = lim.exc
  out = lim.update(-0.5, 2.0 / 3.6, False, True, -0.3, lead_d=2.0)              # the plan eases
  assert out == -0.5 and out > prev                                            # follows it, does not jump to -0.37
