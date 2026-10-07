from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.nnlc_blend import get_nnlc_weight, NNLC_BLEND_V_BP


def test_torque_control_below_30kph():
  for kph in (0, 10, 20, 30):
    assert get_nnlc_weight(kph / 3.6) == 0.0


def test_nnlc_from_45kph():
  for kph in (45, 60, 100, 140):
    assert get_nnlc_weight(kph / 3.6) == 1.0


def test_linear_hand_over():
  mid = (NNLC_BLEND_V_BP[0] + NNLC_BLEND_V_BP[1]) / 2
  assert abs(get_nnlc_weight(mid) - 0.5) < 1e-9
