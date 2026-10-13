from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.lead_brake_boost import LeadBrakeBoost


def lead(a, v=0.0, present=True):
  l = log.RadarState.LeadData.new_message()  # real capnp struct (the field is `present`, not `status`)
  l.present = present
  l.aLeadK = a
  l.vLead = v
  return l


def run(b, n, accel, v, l):
  out = accel
  for _ in range(n):
    out = b.update(accel, v, False, l, True, -3.5)
  return out


def test_no_boost_without_hard_lead_brake():
  b = LeadBrakeBoost()
  assert run(b, 200, -1.5, 15 / 3.6, lead(-1.0)) == -1.5


def test_boost_ramps_to_1p2_between_3_and_30_kmh_and_latches():
  b = LeadBrakeBoost()
  first = b.update(-1.5, 15 / 3.6, False, lead(-3.0), True, -3.5)
  assert -1.5 > first > -1.52  # ramps, no step
  out = run(b, 100, -1.5, 15 / 3.6, lead(0.0))  # lead stopped: aLeadK back to 0, still latched
  assert abs(out - (-1.8)) < 1e-6


def test_2_to_0_kmh_untouched_and_above_33_untouched():
  b = LeadBrakeBoost()
  run(b, 100, -1.5, 15 / 3.6, lead(-3.0))
  assert abs(run(b, 200, -0.5, 1.8 / 3.6, lead(0.0)) - (-0.5)) < 1e-9
  b2 = LeadBrakeBoost()
  assert run(b2, 200, -2.0, 50 / 3.6, lead(-3.0)) == -2.0


def test_release_and_limits():
  b = LeadBrakeBoost()
  run(b, 100, -1.5, 15 / 3.6, lead(-3.0))
  assert b.latched
  b.update(-1.5, 15 / 3.6, False, lead(1.0, v=20 / 3.6), True, -3.5)
  assert not b.latched
  b3 = LeadBrakeBoost()
  assert run(b3, 100, -3.4, 25 / 3.6, lead(-3.0)) == -3.5  # clipped to the accel limit
  assert run(LeadBrakeBoost(), 100, 0.5, 15 / 3.6, lead(-3.0)) == 0.5  # never touches a positive request
  b4 = LeadBrakeBoost()
  run(b4, 100, -1.5, 15 / 3.6, lead(-3.0))
  b4.update(-1.5, 0.0, True, lead(0.0), True, -3.5)
  assert not b4.latched
