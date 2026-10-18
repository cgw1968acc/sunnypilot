from openpilot.sunnypilot.selfdrive.controls.lib.end_force_limit import EndForceLimit, V_LINE, END_ACCEL, RESTOP_END_ACCEL


def test_line_from_2kph_to_minus_030():
  e = EndForceLimit()
  assert e.update(-0.92, 2.8 / 3.6, False, True) == -0.92        # above the line: untouched
  assert e.update(-0.80, 2.0 / 3.6, False, True) == -0.80        # at the line: a_line = -0.80
  out = e.update(-0.70, 1.0 / 3.6, False, True)                  # 18:51: -0.70 at 1.4 km/h asked
  assert abs(out - (END_ACCEL + (-0.80 - END_ACCEL) * 0.5)) < 1e-9
  assert e.update(-0.46, 0.01, False, True) >= END_ACCEL - 0.01 * 0.5 / V_LINE - 1e-9  # at the stop: ~-0.30
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
  assert e.update(-0.9, 1.5 / 3.6, False, True) == -0.9           # the line is anchored where braking began
  out = e.update(-0.9, 0.75 / 3.6, False, True)
  assert abs(out - (END_ACCEL + (-0.9 - END_ACCEL) * 0.5)) < 1e-9
  assert e.update(-0.9, 0.0, False, True) == END_ACCEL


def test_stall_lets_the_line_go_and_reanchors():
  e = EndForceLimit()
  e.update(-0.6, 2.0 / 3.6, False, True, -0.5)
  assert e.update(-0.8, 1.5 / 3.6, False, True, -0.5) > -0.8      # on the line
  for _ in range(30):                                              # rolling at 1.5 km/h without slowing for 0.3 s
    out = e.update(-0.8, 1.5 / 3.6, False, True, 0.0)
  assert out == -0.8                                               # a stall is not the stop: the request passes
  e.update(-0.8, 1.4 / 3.6, False, True, -0.4)                     # slowing again: line anchored at the stall (1.5 km/h)
  out = e.update(-0.8, 0.7 / 3.6, False, True, -0.4)
  assert abs(out - (END_ACCEL + (-0.8 - END_ACCEL) * 0.7 / 1.5)) < 1e-6
  assert e.update(-0.8, 0.0, False, True, -0.4) == END_ACCEL


def _stand_then_creep(e, v_peak):
  for _ in range(60):                                              # stood 0.6 s
    e.update(-1.2, 0.0, True, True)
  for _ in range(100):                                             # creeps off
    e.update(0.3, v_peak, False, True)


def test_restop_after_a_creep_ends_on_minus_026():
  e = EndForceLimit()
  _stand_then_creep(e, 4.0 / 3.6)
  e.update(-0.6, 2.0 / 3.6, False, True)
  out = e.update(-0.6, 1.0 / 3.6, False, True)
  assert abs(out - (RESTOP_END_ACCEL + (-0.6 - RESTOP_END_ACCEL) * 0.5)) < 1e-9
  assert e.update(-0.6, 0.0, False, True) == RESTOP_END_ACCEL


def test_restop_not_after_a_real_launch_or_a_long_creep():
  e = EndForceLimit()
  _stand_then_creep(e, 9.0 / 3.6)                                  # went past 8 km/h: an ordinary stop again
  e.update(-0.6, 2.0 / 3.6, False, True)
  assert e.update(-0.6, 0.0, False, True) == END_ACCEL
  e = EndForceLimit()
  _stand_then_creep(e, 4.0 / 3.6)
  for _ in range(1000):                                            # more than 10 s of moving
    e.update(0.1, 4.0 / 3.6, False, True)
  e.update(-0.6, 2.0 / 3.6, False, True)
  assert e.update(-0.6, 0.0, False, True) == END_ACCEL
