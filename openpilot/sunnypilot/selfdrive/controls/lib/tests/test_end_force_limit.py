import numpy as np
from openpilot.sunnypilot.selfdrive.controls.lib.end_force_limit import EndForceLimit, V_LINE, END_ACCEL, CEIL_BP, CEIL_A


def ceil(kph):
  return float(np.interp(kph / 3.6, CEIL_BP, CEIL_A))


def test_line_from_2kph_to_minus_030():
  e = EndForceLimit()
  assert e.update(-0.92, 2.8 / 3.6, False, True) == -0.92        # above the 2 km/h line, under the ceiling: untouched
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
  for kph, want in ((5.0, -2.0), (4.0, -1.6), (3.0, -1.2), (2.0, -0.60), (1.4, -0.468), (1.0, -0.38), (0.5, -0.36), (0.0, END_ACCEL)):
    out = e.update(-2.7, kph / 3.6, False, True)
    assert abs(out - want) < 0.005, (kph, out)
  e2 = EndForceLimit()
  assert e2.update(-0.65, 5.0 / 3.6, False, True) == -0.65                    # an ordinary end (END curve) never touches it
  assert e2.update(-0.60, 3.0 / 3.6, False, True) == -0.60
  e3 = EndForceLimit()                                                        # into the standstill hold without a step
  e3.update(-2.7, 1.0 / 3.6, False, True)
  assert e3.update(-2.7, 0.0, False, True) == END_ACCEL
  out = e3.update(-1.2, 0.0, True, True)
  assert abs(out - (END_ACCEL - 0.005)) < 1e-9
