import openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.nnlc_blend as NB
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.nnlc_blend import get_nnlc_weight, NNLC_BLEND_V_BP


def test_blend_off_means_nnlc_at_every_speed():
  # tncr (Corolla Cross) 2026-10-15: the blend is off, as in tncr19
  assert NB.NNLC_BLEND_ENABLED is False
  for kph in (0, 10, 20, 30, 45, 100):
    assert get_nnlc_weight(kph / 3.6) == 1.0


def test_blend_table_when_enabled(monkeypatch):
  monkeypatch.setattr(NB, "NNLC_BLEND_ENABLED", True)
  for kph in (0, 10, 20, 30):
    assert get_nnlc_weight(kph / 3.6) == 0.0
  for kph in (45, 60, 100, 140):
    assert get_nnlc_weight(kph / 3.6) == 1.0
  mid = (NNLC_BLEND_V_BP[0] + NNLC_BLEND_V_BP[1]) / 2
  assert abs(get_nnlc_weight(mid) - 0.5) < 1e-9
