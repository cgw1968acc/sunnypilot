from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.long_mpc_ext import LongMpcObstacleExt, RELAXED_JERK_FACTOR

P = log.LongitudinalPersonality


def test_relaxed_uses_the_relaxed_jerk_factor():
  assert LongMpcObstacleExt.jerk_factor(P.relaxed, 1.0) == RELAXED_JERK_FACTOR == 3.0


def test_standard_and_aggressive_keep_stock():
  assert LongMpcObstacleExt.jerk_factor(P.standard, 1.0) == 1.0
  assert LongMpcObstacleExt.jerk_factor(P.aggressive, 0.5) == 0.5
