import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.accel_controller.accel_controller import (
  AccelController, AccelProfile, MAX_ACCEL_BREAKPOINTS, MAX_ACCEL_PROFILES,
)

class TestEcoLeadBoost:
  """Eco lead pull-away boost (driver 2026-10-01 night)."""
  @staticmethod
  def _ac(profile):
    from types import SimpleNamespace
    ac = AccelController.__new__(AccelController)
    ac._profile, ac._enabled, ac._boost = profile, True, 0.0
    ac._cruise_decel = ac._cruise_decel_target = None
    return ac, SimpleNamespace

  def test_no_lead_unchanged(self):
    ac, _ = self._ac(AccelProfile.eco)
    v = 15.0
    base = float(np.interp(v, MAX_ACCEL_BREAKPOINTS, MAX_ACCEL_PROFILES[AccelProfile.eco]))
    for _ in range(100):
      assert ac.get_max_accel(v, False, None) == base

  def test_pulling_away_lead_boosts_smoothly_within_normal(self):
    ac, NS = self._ac(AccelProfile.eco)
    v = 15.0
    lead = NS(present=True, modelProb=0.9, dRel=30.0, vLead=v + 3.0)
    base = float(np.interp(v, MAX_ACCEL_BREAKPOINTS, MAX_ACCEL_PROFILES[AccelProfile.eco]))
    normal = float(np.interp(v, MAX_ACCEL_BREAKPOINTS, MAX_ACCEL_PROFILES[AccelProfile.normal]))
    out = [ac.get_max_accel(v, False, lead) for _ in range(60)]
    steps = np.diff([base] + out)
    assert max(steps) <= 0.25 * 0.05 + 1e-9          # eased in
    assert out[-1] > base + 0.2 and out[-1] <= normal + 1e-9
    lead.present = False
    out2 = [ac.get_max_accel(v, False, lead) for _ in range(60)]
    assert min(np.diff([out[-1]] + out2)) >= -0.5 * 0.05 - 1e-9   # eased out
    assert abs(out2[-1] - base) < 1e-9

  def test_no_boost_at_80_kph_or_for_a_closing_lead(self):
    ac, NS = self._ac(AccelProfile.eco)
    v = 80 / 3.6
    lead = NS(present=True, modelProb=0.9, dRel=30.0, vLead=v + 3.0)
    base = float(np.interp(v, MAX_ACCEL_BREAKPOINTS, MAX_ACCEL_PROFILES[AccelProfile.eco]))
    assert abs([ac.get_max_accel(v, False, lead) for _ in range(40)][-1] - base) < 1e-9
    ac, NS = self._ac(AccelProfile.eco)
    v = 15.0
    lead = NS(present=True, modelProb=0.9, dRel=30.0, vLead=v - 1.0)
    base = float(np.interp(v, MAX_ACCEL_BREAKPOINTS, MAX_ACCEL_PROFILES[AccelProfile.eco]))
    assert abs([ac.get_max_accel(v, False, lead) for _ in range(40)][-1] - base) < 1e-9
