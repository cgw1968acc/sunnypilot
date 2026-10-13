from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.lead_brake_boost import LeadBrakeBoost


def lead(a, v=0.0, d=10.0, present=True):
  l = log.RadarState.LeadData.new_message()  # real capnp struct (the field is `present`, not `status`)
  l.present = present
  l.aLeadK = a
  l.vLead = v
  l.dRel = d
  return l


def run(b, n, accel, v, l):
  out = accel
  for _ in range(n):
    out = b.update(accel, v, False, l, True, -3.5)
  return out


V15 = 15 / 3.6


def test_no_boost_for_an_ordinary_lead_stop():
  # lead brakes hard but is far: TTC 10 m / (4.2 - 1.0) m/s = 3.1 s ... use 20 m -> 6.3 s
  assert run(LeadBrakeBoost(), 200, -1.5, V15, lead(-2.5, v=1.0, d=20.0)) == -1.5
  # closing too slowly (5 km/h)
  assert run(LeadBrakeBoost(), 200, -1.5, V15, lead(-3.0, v=V15 - 5 / 3.6, d=3.0)) == -1.5
  # lead brakes only moderately
  assert run(LeadBrakeBoost(), 200, -1.5, V15, lead(-1.5, v=0.0, d=5.0)) == -1.5


def test_factor_follows_the_hardest_lead_decel_and_latches():
  for a_lead, gain in ((-2.0, 1.10), (-3.0, 1.20), (-3.61, 1.261), (-5.0, 1.30)):
    b = LeadBrakeBoost()
    first = b.update(-1.5, V15, False, lead(a_lead, v=0.0, d=8.0), True, -3.5)
    assert -1.5 > first > -1.52  # ramps, no step
    out = run(b, 100, -1.5, V15, lead(0.0, v=0.0, d=6.0))  # lead stopped: aLeadK back to 0, still latched
    assert abs(out - (-1.5 * gain)) < 2e-3, (a_lead, out)


def test_speed_window():
  b = LeadBrakeBoost()
  run(b, 100, -1.5, V15, lead(-3.0, d=8.0))
  assert abs(run(b, 200, -0.5, 1.8 / 3.6, lead(0.0, d=2.0)) - (-0.5)) < 1e-9  # 2-0 km/h untouched
  b2 = LeadBrakeBoost()
  assert run(b2, 200, -2.0, 40 / 3.6, lead(-3.0, d=15.0)) == -2.0  # above 30 km/h untouched
  b3 = LeadBrakeBoost()
  out = run(b3, 200, -2.0, 25 / 3.6, lead(-3.0, d=10.0))  # 25 km/h: half way in (x1.10)
  assert abs(out - (-2.2)) < 1e-6


def test_release_and_limits():
  b = LeadBrakeBoost()
  run(b, 100, -1.5, V15, lead(-3.0, d=8.0))
  assert b.latched
  b.update(-1.5, V15, False, lead(1.0, v=20 / 3.6, d=8.0), True, -3.5)
  assert not b.latched
  assert run(LeadBrakeBoost(), 100, -3.4, V15, lead(-3.0, d=8.0)) == -3.5  # clipped to the accel limit
  assert run(LeadBrakeBoost(), 100, 0.5, V15, lead(-3.0, d=8.0)) == 0.5  # never touches a positive request
  b4 = LeadBrakeBoost()
  run(b4, 100, -1.5, V15, lead(-3.0, d=8.0))
  b4.update(-1.5, 0.0, True, lead(0.0), True, -3.5)
  assert not b4.latched
