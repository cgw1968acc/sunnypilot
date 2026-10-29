from openpilot.sunnypilot.selfdrive.controls.lib.e2e_stop.e2e_stop_anchor import E2eStopAnchor, model_stop_x, RETREAT_DB, STOP_D

DT = 0.05


def test_model_stop_x():
  assert model_stop_x([0, 5, 10, 15], [5.0, 3.0, 0.2, 0.0]) == 10.0
  assert model_stop_x([0, 5, 10], [5.0, 3.0, 1.0]) is None


def test_ordinary_stop_is_the_model_unchanged():
  # the stop point stays put on the ground: output == model accel, stop request untouched
  a = E2eStopAnchor()
  v, x_line = 6.0, 30.0
  while v > 0.2:
    r = a.update(True, v, DT, x_line - a.odo, -0.7, None)
    assert r == (-0.7, True)
    v -= 0.7 * DT


def test_drift_back_is_not_followed_but_never_softer_than_landing_needs():
  a = E2eStopAnchor()
  r = a.update(True, 5.4, DT, 27.5, -0.64, None)           # model commits at 27.5 m
  assert r == (-0.64, True)
  # route be: the model's point creeps back ~4-5 m per second while the car slows
  v, x = 5.4, 27.5
  for _ in range(40):                                        # 2 s
    v -= 0.75 * DT
    x -= 0.25 + v * DT                                       # 0.25 m/frame drift + the car's own travel
    r = a.update(True, v, DT, x, -0.83, None)
  assert a.active
  d_land = a.stop_at - a.odo - RETREAT_DB
  need = v ** 2 / (2 * d_land)
  assert -0.83 < r[0] <= -need + 1e-9                      # softer than the model, firm enough to land
  assert r[1] is False                                      # no early stop request


def test_sudden_retreat_or_firm_request_is_followed():
  a = E2eStopAnchor()
  a.update(True, 8.0, DT, 40.0, -0.5, None)
  r = a.update(True, 8.0, DT, 20.0, -0.9, None)             # 20 m jump in one frame: something new
  assert r == (-0.9, True) and not a.active
  b = E2eStopAnchor()
  b.update(True, 8.0, DT, 40.0, -0.5, None)
  b.update(True, 8.0, DT, 37.0, -0.6, None)
  r = b.update(True, 8.0, DT, 34.0, -1.6, None)             # firm request
  assert r == (-1.6, True)


def test_lead_near_or_not_in_control_does_nothing():
  a = E2eStopAnchor()
  assert a.update(True, 6.0, DT, 25.0, -0.7, 28.0) is None   # radar lead near the stop
  assert a.update(False, 6.0, DT, 25.0, -0.7, None) is None
  assert a.update(True, 6.0, DT, None, 0.2, None) is None     # model not stopping


def test_end_is_handed_back_to_the_model():
  a = E2eStopAnchor()
  a.update(True, 3.0, DT, 6.0, -0.5, None)
  a.odo += 6.0 - RETREAT_DB - STOP_D + 0.1                  # within STOP_D of the landing point
  r = a.update(True, 0.8, DT, 0.3, -0.4, None)
  assert r == (-0.4, True)


def test_stopped_car_is_never_pulled_off_again():
  # route 77 (10-23): stopped short of the line with the anchored point still ~4.7 m ahead
  a = E2eStopAnchor()
  a.update(True, 5.0, DT, 30.0, -0.4, None)
  v, x = 5.0, 30.0
  while v > 0.35:
    v -= 0.6 * DT
    x -= 0.2 + v * DT
    a.update(True, v, DT, max(x, 0.0), -0.6, None)
  r = a.update(True, 0.1, DT, 0.0, -0.3, None)
  assert r == (-0.3, True)


def test_soft_end_gets_room_before_the_point():
  # the model and the end layers take over once a soft end (A_END) from this speed needs the remaining distance
  from openpilot.sunnypilot.selfdrive.controls.lib.e2e_stop.e2e_stop_anchor import A_END
  a = E2eStopAnchor()
  a.update(True, 6.0, DT, 30.0, -0.5, None)
  a.odo += 30.0 - RETREAT_DB - 6.0 ** 2 / (2 * A_END) + 0.05   # just inside that distance
  r = a.update(True, 6.0, DT, 2.0, -0.9, None)
  assert r == (-0.9, True) and not a.active
