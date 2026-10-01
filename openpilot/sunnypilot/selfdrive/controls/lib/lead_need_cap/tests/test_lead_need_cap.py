from openpilot.sunnypilot.selfdrive.controls.lib.lead_need_cap.lead_need_cap import (
  LeadNeedCap, lead_need, NEED_MARGIN, RELEASE_RATE,
)
DT = 0.05


def test_slowing_lead_mpc_overbrake_is_eased_to_the_need():
  # ego 50 km/h, lead 30 m ahead at 40 km/h slowing gently: the need is small, the MPC's -2.0 is eased toward it
  cap = LeadNeedCap(DT)
  v, d, vl, al = 13.9, 30.0, 11.1, -0.2
  need = lead_need(v, d, vl, al)
  assert need is not None and need < 0.5
  outs = [cap.update(True, v, True, d, vl, al, -2.0) for _ in range(60)]
  assert outs[0] >= -2.0 and abs(outs[-1] + NEED_MARGIN * need) < 1e-6
  steps = [b - a for a, b in zip(outs, outs[1:])]
  assert max(steps) <= RELEASE_RATE * DT + 1e-9          # eased, never a step


def test_never_harder_than_mpc_and_lighter_mpc_passes():
  cap = LeadNeedCap(DT)
  for a in (-0.25, 0.5, -0.1):
    assert cap.update(True, 13.9, True, 30.0, 11.1, -0.2, a) == a     # lighter than the need: untouched
  assert cap.update(True, 13.9, True, 30.0, 11.1, -0.2, -0.6) >= -0.6  # capped, never harder


def test_emergency_untouched():
  cap = LeadNeedCap(DT)
  # lead braking hard at short range: need > NEED_MAX or TTC < TTC_MIN -> MPC passes
  assert cap.update(True, 13.9, True, 12.0, 5.0, -3.0, -3.0) == -3.0
  cap = LeadNeedCap(DT)
  assert cap.update(True, 13.9, True, 10.0, 8.0, 0.0, -2.5) == -2.5   # TTC 1.7 s


def test_lead_starts_braking_harder_need_grows_at_once():
  cap = LeadNeedCap(DT)
  v, d, vl = 13.9, 35.0, 12.0
  for _ in range(40):
    a0 = cap.update(True, v, True, d, vl, -0.1, -1.5)
  a1 = cap.update(True, v, True, d, vl, -2.5, -2.5)   # lead now braking -2.5: stop projection, more brake allowed at once
  assert a1 < a0 - 0.5


def test_slow_lead_left_to_stop_governor():
  cap = LeadNeedCap(DT)
  assert cap.update(True, 8.0, True, 20.0, 1.0, -1.0, -2.0) == -2.0
