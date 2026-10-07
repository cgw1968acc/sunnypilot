from opendbc.car.common.conversions import Conversions as CV
from opendbc.sunnypilot.car.toyota.carstate_ext import set_speed_target, cluster_speed, SET_SPEED_SCREEN_MARGIN_KPH


def test_dash_reads_the_set_speed():
  # route 00000110: UI 100 -> PCM 93 -> dash 99.1; the target now puts the dash just above the set speed
  for ui in (30, 60, 85, 98, 100, 120):
    dash = cluster_speed(set_speed_target(ui)) * CV.MS_TO_KPH
    assert abs(dash - (ui + SET_SPEED_SCREEN_MARGIN_KPH)) < 1e-6


def test_100_is_about_94_true():
  assert abs(set_speed_target(100) * CV.MS_TO_KPH - 94.12) < 0.01
