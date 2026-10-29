# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
from types import SimpleNamespace as NS
import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.low_speed_straight_hold import LowSpeedStraightHold, \
  PLAN_K_MAX

T = np.linspace(0, 10, 33) ** 2 / 10


def model(k_plan):
  return NS(orientationRate=NS(t=T, z=np.full(33, k_plan * 5.0)), velocity=NS(x=np.full(33, 5.0)))


def cs(v_kph, angle=0.0, blinker=False, pressed=False):
  return NS(vEgo=v_kph / 3.6, steeringAngleDeg=angle, leftBlinker=blinker, rightBlinker=False, steeringPressed=pressed)


def run(h, ks, md, c, active=True):
  return [h.update(k, md, c, active, False) for k in ks]


def test_straight_creep_holds_the_wheel():
  h = LowSpeedStraightHold(0.01)
  ks = 0.0025 * np.sign(np.sin(np.arange(1500) * 0.01 * 2 * np.pi / 2.5))  # +/-2.5e-3 hops every 1.25 s
  out = np.array(run(h, ks, model(0.001), cs(5)))
  assert np.abs(out[200:]).max() < 0.0006                                 # vs 0.0025 asked


def test_turn_is_not_touched():
  h = LowSpeedStraightHold(0.01)
  out = run(h, [0.05] * 200, model(0.05), cs(8, angle=90))
  assert all(abs(o - 0.05) < 1e-12 for o in out)


def test_lets_go_quickly_when_a_turn_starts():
  h = LowSpeedStraightHold(0.01)
  run(h, [0.0] * 200, model(0.0), cs(5))
  out = run(h, [0.03] * 50, model(PLAN_K_MAX * 2), cs(5))                 # the plan bends
  assert out[0] > 0.0 and abs(out[-1] - 0.03) < 0.002                     # most of it within 0.5 s, no jump
  assert all(b >= a - 1e-12 for a, b in zip(out, out[1:], strict=False))


def test_blinker_hands_and_speed_disable():
  for c in (cs(5, blinker=True), cs(5, pressed=True), cs(25)):
    h = LowSpeedStraightHold(0.01)
    out = run(h, [0.002] * 200, model(0.0), c)
    assert abs(out[-1] - 0.002) < 1e-12
  h = LowSpeedStraightHold(0.01)
  assert run(h, [0.002] * 200, model(0.0), cs(5), active=False)[-1] == 0.002
