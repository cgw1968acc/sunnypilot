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
URGENT = lead(-3.0, v=0.0, d=8.0)


def test_no_boost_for_an_ordinary_lead_stop():
  assert run(LeadBrakeBoost(), 300, -1.5, V15, lead(-2.5, v=1.0, d=20.0)) == -1.5   # TTC too long
  assert run(LeadBrakeBoost(), 300, -1.5, V15, lead(-3.0, v=V15 - 5 / 3.6, d=3.0)) == -1.5  # closing too slowly
  assert run(LeadBrakeBoost(), 300, -1.5, V15, lead(-1.5, v=0.0, d=5.0)) == -1.5   # lead brakes only moderately


def test_not_in_the_light_first_phase():
  b = LeadBrakeBoost()
  assert run(b, 300, -0.8, V15, URGENT) == -0.8      # light request: never boosted
  b = LeadBrakeBoost()
  assert run(b, 40, -1.5, V15, URGENT) == -1.5       # firm but the first 0.4 s of braking: not yet
  out = run(b, 100, -1.5, V15, URGENT)               # after 0.5 s it ramps in (0.4 s) to x1.2
  assert abs(out - (-1.8)) < 1e-6


def test_held_after_the_peak_and_faded_at_the_end():
  b = LeadBrakeBoost()
  run(b, 200, -2.5, V15, URGENT)
  assert abs(run(b, 10, -0.9, 8 / 3.6, lead(0.0, d=4.0)) - (-1.08)) < 1e-6  # easing after the peak: still x1.2
  assert abs(run(b, 300, -0.5, 1.8 / 3.6, lead(0.0, d=2.0)) - (-0.5)) < 1e-9  # 2-0 km/h untouched


def test_release_and_limits():
  b = LeadBrakeBoost()
  run(b, 200, -1.5, V15, URGENT)
  assert b.latched
  b.update(-1.5, V15, False, lead(1.0, v=20 / 3.6, d=8.0), True, -3.5)
  assert not b.latched
  assert run(LeadBrakeBoost(), 200, -3.4, V15, URGENT) == -3.5  # clipped to the accel limit
  assert run(LeadBrakeBoost(), 200, 0.5, V15, URGENT) == 0.5    # never touches a positive request
  b4 = LeadBrakeBoost()
  run(b4, 200, -1.5, V15, URGENT)
  b4.update(-1.5, 0.0, True, lead(0.0), True, -3.5)
  assert not b4.latched
