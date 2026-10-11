import unittest
from types import SimpleNamespace

from opendbc.sunnypilot.car.toyota.brake_onset import (BrakeHandoverFeedforward, HANDOVER_EXTRA_MAX,
                                                       HANDOVER_EXTRA_FRAC, HANDOVER_RATE, PID_BLEED_NEG, PID_BLEED_POS,
                                                       LOW_EXTRA, FIRM_TAU)

DT = 0.03
class TestHandoverFeedforward(unittest.TestCase):
  def run_steady(self, req, v_kph, n=200):
    c = BrakeHandoverFeedforward(DT)
    for _ in range(n):
      c.update(req, v_kph / 3.6)
    return c

  def test_full_extra_in_the_dip(self):
    c = self.run_steady(-0.84, 12.0)   # route 0000010e: asked -0.84 at 12-9 km/h, the car gave -0.61
    self.assertAlmostEqual(c.extra, HANDOVER_EXTRA_MAX, places=9)
    self.assertAlmostEqual(c.apply(-0.84), -0.84 - HANDOVER_EXTRA_MAX, places=9)

  def test_lighter_request_gets_a_quarter(self):
    self.assertAlmostEqual(self.run_steady(-0.5, 12.0).extra, HANDOVER_EXTRA_FRAC * 0.5, places=9)

  def test_fades_at_the_edges(self):
    self.assertAlmostEqual(self.run_steady(-1.0, 8.0).extra, 0.5 * HANDOVER_EXTRA_MAX + 0.25 * LOW_EXTRA[2], places=6)
    self.assertEqual(self.run_steady(-1.0, 2.5).extra, 0.0)
    self.assertEqual(self.run_steady(-1.0, 20.0).extra, 0.0)
    self.assertEqual(self.run_steady(-0.25, 12.0).extra, 0.0)

  def test_low_speed_extra(self):
    # route 0000011c 00:19:10: asked -0.67 at 5-4 km/h, the car gave -0.28
    self.assertAlmostEqual(self.run_steady(-0.67, 5.0).extra, 0.25, places=6)
    self.assertAlmostEqual(self.run_steady(-0.67, 6.0).extra, 0.25, places=6)
    self.assertAlmostEqual(self.run_steady(-0.62, 4.0).extra, 0.15, places=6)
    self.assertEqual(self.run_steady(-0.6, 3.0).extra, 0.0)   # the last km/h already brakes on target
    self.assertEqual(self.run_steady(-0.25, 5.0).extra, 0.0)  # not for a request lighter than HANDOVER_MIN_REQUEST
    self.assertEqual(self.run_steady(-1.5, 5.0).extra, 0.0)   # nor for a firm one

  def test_fades_out_for_a_firm_request(self):
    self.assertEqual(self.run_steady(-1.5, 12.0).extra, 0.0)   # 0000010d: at -1.46 the car delivered -1.72
    self.assertAlmostEqual(self.run_steady(-1.25, 12.0).extra, 0.5 * HANDOVER_EXTRA_MAX, places=6)

  def run_firm(self, req, v_kph, a_ego, n=200):
    c = BrakeHandoverFeedforward(DT)
    for _ in range(n):
      c.update(req, v_kph / 3.6, a_ego=a_ego)
    return c

  def test_firm_request_short_gets_the_extra(self):
    # route 00000133 15:04:52: asked -1.55 at 14 km/h, the car gave -1.25
    self.assertAlmostEqual(self.run_firm(-1.55, 14.0, -1.25).extra, HANDOVER_EXTRA_MAX, places=6)
    # half-firm request: moderate part + firm part add up to the full extra
    self.assertAlmostEqual(self.run_firm(-1.25, 12.0, -1.0).extra, HANDOVER_EXTRA_MAX, places=6)

  def test_firm_request_on_target_or_over_gets_nothing(self):
    self.assertEqual(self.run_firm(-1.5, 12.0, -1.72).extra, 0.0)  # 0000010d: over-delivered
    self.assertEqual(self.run_firm(-1.5, 12.0, -1.43).extra, 0.0)  # inside the gate's dead zone

  def test_firm_extra_band(self):
    self.assertEqual(self.run_firm(-1.6, 5.0, -1.2).extra, 0.0)    # the end of the stop stays on the end table
    self.assertEqual(self.run_firm(-1.6, 22.0, -1.2).extra, 0.0)
    self.assertEqual(self.run_firm(-2.6, 12.0, -2.0).extra, 0.0)   # emergency-hard: left to the planner
    self.assertAlmostEqual(self.run_firm(-1.6, 18.5, -1.2).extra, 0.5 * HANDOVER_EXTRA_MAX, places=6)

  def test_over_delivering_car_gets_no_extra(self):
    # route 00000134 15:32:12: asked -0.97 at 12 km/h, the car gave -1.35; 15:32:14 asked -0.72 at 6 km/h, gave -1.03
    self.assertEqual(self.run_firm(-0.97, 12.0, -1.35).extra, 0.0)
    self.assertEqual(self.run_firm(-0.72, 6.0, -1.03).extra, 0.0)

  def test_extra_holds_once_the_car_responds(self):
    # the car is 0.25 short on its own and delivers the extra 1:1: the extra must stay, not pulse
    c = BrakeHandoverFeedforward(DT)
    extras = []
    for _ in range(300):
      c.update(-0.84, 12 / 3.6, a_ego=-0.84 + 0.25 - c.extra)
      extras.append(c.extra)
    self.assertAlmostEqual(extras[-1], HANDOVER_EXTRA_MAX, places=6)
    self.assertTrue(all(b >= a - 1e-9 for a, b in zip(extras[100:], extras[101:])))

  def test_firm_gate_ignores_a_short_lag(self):
    # a request that just stepped firmer: the ~0.3 s actuator lag barely opens the gate
    c = BrakeHandoverFeedforward(DT)
    for _ in range(int(0.3 / DT)):
      c.update(-1.6, 12 / 3.6, a_ego=-1.3)
    self.assertLess(c.extra, 0.06)

  def test_gradual(self):
    c = BrakeHandoverFeedforward(DT)
    prev = 0.0
    for _ in range(50):
      e = c.update(-1.0, 12 / 3.6)
      self.assertLessEqual(e - prev, HANDOVER_RATE * DT + 1e-9)
      prev = e

class TestPidHold(unittest.TestCase):
  def test_braking_integral_bleeds_and_freezes_below_9kph(self):
    c = BrakeHandoverFeedforward(DT)
    pid = SimpleNamespace(i=-0.15)
    self.assertTrue(c.condition_pid(pid, -0.6, 5 / 3.6))
    self.assertAlmostEqual(pid.i, -0.15 + PID_BLEED_NEG * DT, places=9)
    for _ in range(100):
      c.condition_pid(pid, -0.6, 5 / 3.6)
    self.assertEqual(pid.i, 0.0)

  def test_gas_integral_left_from_a_launch_bleeds_fast(self):
    c = BrakeHandoverFeedforward(DT)
    pid = SimpleNamespace(i=0.3)
    c.condition_pid(pid, -0.2, 3 / 3.6)
    self.assertAlmostEqual(pid.i, 0.3 - PID_BLEED_POS * DT, places=9)

  def test_untouched_above_9kph_or_when_not_braking(self):
    c = BrakeHandoverFeedforward(DT)
    pid = SimpleNamespace(i=-0.15)
    self.assertFalse(c.condition_pid(pid, -0.6, 12 / 3.6))
    self.assertFalse(c.condition_pid(pid, 0.3, 3 / 3.6))
    self.assertEqual(pid.i, -0.15)


if __name__ == "__main__":
  unittest.main()
