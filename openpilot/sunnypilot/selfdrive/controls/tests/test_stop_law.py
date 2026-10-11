"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.sunnypilot.selfdrive.controls.lib.stop_law as SL
from openpilot.sunnypilot.selfdrive.controls.lib.stop_law import StopLaw

DT = 0.05
LAG = 0.3


def kph(v):
  return v / 3.6


def drive(v0, d0, lead_v0, lead_a, a_plan=-0.5, lead_go_t=None, secs=25.0, law=None):
  """ego follows the law's output (or a_plan when the law returns None) through a 0.3 s lag; the lead brakes at lead_a
  to a stop (or drives off again at lead_go_t). Returns per-step rows: t, v, delivered decel, command, gap, active."""
  law = law or StopLaw(DT)
  v, a, x, xl, vl = v0, 0.0, 0.0, d0, lead_v0
  prev, rows = 0.0, []
  for i in range(int(secs / DT)):
    t = i * DT
    al = lead_a if lead_go_t is None or t < lead_go_t else 1.0
    vl = max(vl + al * DT, 0.0)
    xl += vl * DT
    al_seen = al if vl > 0 else 0.0
    out = law.update(True, v, v < 0.01, True, xl - x, vl, al_seen, a_plan, prev)
    cmd = a_plan if out is None else out
    prev = cmd
    a += (cmd - a) * DT / LAG
    v = max(v + a * DT, 0.0)
    x += v * DT
    rows.append((t, v, -a, cmd, xl - x, law.active))
    if v <= 0.0 and vl <= 0.0:
      break
  return np.array(rows, dtype=float)


def test_following_a_moving_lead_is_left_to_the_mpc():
  r = drive(kph(50), 27.0, kph(50), 0.0, a_plan=0.0, secs=5.0)
  assert not r[:, 5].any()
  assert np.all(r[:, 3] == 0.0)


def test_lead_braking_to_a_stop_soft_build_soft_end():
  # route 00000133 15:04: 48 km/h, 26 m behind, the lead braked at about -2 to a stop
  r = drive(kph(48), 26.0, kph(45), -2.0)
  r = r[r[:, 5] > 0]  # while the law drives (at the standstill the stopping controller's hold takes over)
  cmd = -r[:, 3]
  jerk = np.diff(cmd) / DT
  assert r[:, 5].any()
  assert jerk.max() <= SL.J_BUILD + 1e-6           # smooth build
  assert -jerk.min() <= SL.J_RELEASE + 1e-6        # smooth release
  assert cmd.max() < 3.0                           # no need for a hard peak here
  assert cmd[int(0.3 / DT)] - cmd[0] < 0.45          # starts from the lightest touch (jerk ramps up, snap-limited)
  assert r[-1, 2] < 0.4                            # stops at the lightest force: no jolt
  assert 3.0 <= r[-1, 4] <= 4.5                    # 3.5 m behind, give or take
  # the peak comes in the first part of the braking distance
  k = int(np.argmax(cmd)); s = np.cumsum(r[:, 1] * DT)
  assert s[k] / s[-1] < 0.6


def test_stopped_car_found_late_uses_the_2_5_m_mode():
  # route 00000134 15:29: 59 km/h, a stopped car at 51 m
  r = drive(kph(59), 51.0, 0.0, 0.0)
  r = r[r[:, 5] > 0]
  law_gap = r[-1, 4]
  assert 2.0 <= law_gap < 3.3
  assert r[-1, 2] < 0.6
  assert (-r[:, 3]).max() < 4.5


def test_lead_drives_on_hands_back_smoothly():
  r = drive(kph(40), 25.0, kph(40), -2.0, a_plan=0.3, lead_go_t=3.0, secs=8.0)
  assert r[:, 5].any() and not r[-1, 5]
  jerk = np.diff(r[:, 3]) / DT
  after = r[1:, 0] > 3.0
  assert np.abs(jerk[after]).max() <= max(SL.HANDBACK_JERK, SL.J_RELEASE) + 1e-6
  assert abs(r[-1, 3] - 0.3) < 1e-6  # back on the planner's target


def test_disable_flag(tmp_path, monkeypatch):
  flag = tmp_path / "tn_no_stop_law"
  flag.write_text("")
  monkeypatch.setattr(SL, "DISABLE_FLAG", str(flag))
  r = drive(kph(48), 26.0, kph(45), -2.0, secs=3.0)
  assert not r[:, 5].any()


def test_not_above_v_max_or_without_permission():
  law = StopLaw(DT)
  assert law.update(True, kph(90), False, True, 40.0, 0.0, 0.0, -1.0, 0.0) is None
  assert law.update(False, kph(40), False, True, 20.0, 0.0, 0.0, -1.0, 0.0) is None
