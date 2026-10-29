# Copyright (c) 2026 cgw1968acc
# Developed under the direction of cgw1968acc with AI assistance (Claude, Anthropic).
# Design, specifications, road testing and tuning by cgw1968acc.
# Consultant: rav4kumar (Kumar).
# Part of a sunnypilot-based project, distributed under the sunnypilot license (see LICENSE.md and NOTICE).
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.selfdrive.controls.lib.latcontrol import LatControl

# Speed-split torque controller (owner 2026-10-12, after a v0 highway test: "v0 feels steadier on the highway, really fewer
# corrections; can the highway use v0 and below 60 km/h switch to v1?"). Route 0000013d (v1, 94 km/h): the side-to-side
# corrections in gentle curves were ~95% the model's own action (0.144 m/s^2 in the 0.15-0.6 Hz band, tnpb2 path
# corrections 0.003-0.010), so the controller only changes how the car and the model loop together; route 00000109 (V0)
# against 00000110 (v1) at 92-94 km/h had 35% less slow steering correction. Low speed keeps v1 and its tnpb2 tuning
# (KP schedule, turn-exit limiter, lane trim, path corrections).
# Both controllers run every frame with the same inputs, so the one taking over is already warm (integrator, filters,
# request buffer). The output torque is cross-faded by speed: v1 below SPLIT_V_BP[0], v0 above SPLIT_V_BP[1].
# Their PID integrators are kept as ONE shared state (weighted by the same cross-fade every frame): the controller that
# does not drive the car sees its error never close, so on its own its integrator winds up (open-loop replay of route
# 0000013d: the idle v0 reached 0.6-0.9 torque at 85-94 km/h against a logged -0.1) and would kick the wheel when it
# takes over. With the shared integrator the one taking over starts from the integral that actually steered the car.
SPLIT_ENABLED = False  # False = plain v1 as before (owner 2026-10-13: back to plain v1, see the commit)
# Lane centering on the v0 side (owner 2026-10-13: "on the highway I want it to centre a bit quicker, without turning
# into side-to-side swaying; keep the FRAC correction too"). The tnpb2 path corrections live in v1 (correct_desired_curvature),
# so above 65 km/h, where v0 drives alone, the car had no centering at all and only the model's path held it in the lane;
# the low-speed lane centering (designed up to 70 km/h) also faded with v1 from 55 km/h. v1 still computes them every frame,
# so hand v0 the same centering curvature (highway LaneCentering, full from 80 km/h, + low-speed lane centering, up to
# 70 -> 80 km/h), added after v0's own curve outward bias (the FRAC table, same function as v1's). Gains are the road-tuned
# v1 ones (KP 0.35 / KD 0.9 / KI 0.05, jerk 0.3 m/s^3). Open loop on route 0000013f (v0, > 80 km/h, 266 s): correction
# p95 0.23 m/s^2, 0.15-0.6 Hz desired lateral accel +7%; route 0000013d (v1 with the same centering) against 13f (v0
# without): offset > 0.3 m 0.6% vs 1.4% of the time, std 0.091 vs 0.104 m. Turn-exit limiter / trim stay v1 only.
V0_CENTERING_ENABLED = True  # False = v0 without centering, as in edb5f107dd
SPLIT_V_BP = [55.0 / 3.6, 65.0 / 3.6]  # m/s
SPLIT_V0_W = [0.0, 1.0]


def get_v0_weight(v_ego: float) -> float:
  return float(np.interp(v_ego, SPLIT_V_BP, SPLIT_V0_W))


class _ExtensionFanOut:
  """controlsd calls LaC.extension.update_limits / update_model_v2 / update_lateral_lag: pass them to both controllers."""
  def __init__(self, *extensions):
    self.extensions = extensions

  def update_limits(self):
    for e in self.extensions:
      e.update_limits()

  def update_model_v2(self, model_v2):
    for e in self.extensions:
      e.update_model_v2(model_v2)

  def update_lateral_lag(self, lag):
    for e in self.extensions:
      e.update_lateral_lag(lag)

  def __getattr__(self, name):
    return getattr(self.extensions[0], name)


class LatControlTorqueSplit(LatControl):
  def __init__(self, lac_v1, lac_v0, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.v1 = lac_v1
    self.v0 = lac_v0
    self.extension = _ExtensionFanOut(lac_v1.extension, lac_v0.extension)
    self.v0_weight = 0.0

  def _centering_curvature(self) -> float:
    """The lane-centering part of v1's path corrections of this frame (v1.update ran first)."""
    pc = getattr(self.v1.extension, 'path_corrections', None)
    if pc is None:
      return 0.0
    c = pc.lane_centering.correction
    if pc.low_centering is not None:
      c += pc.low_centering.correction
    return float(c)

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, friction):
    self.v1.update_torque_parameters(latAccelFactor, latAccelOffset, friction)
    self.v0.update_torque_parameters(latAccelFactor, latAccelOffset, friction)

  def reset(self):
    super().reset()
    self.v1.reset()
    self.v0.reset()

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    steer_v1, out_v1, log_v1 = self.v1.update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose,
                                              curvature_limited, lat_delay)
    self.v0.centering_curvature = self._centering_curvature() if V0_CENTERING_ENABLED else 0.0
    steer_v0, out_v0, log_v0 = self.v0.update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose,
                                              curvature_limited, lat_delay)
    w = get_v0_weight(CS.vEgo)
    self.v0_weight = w
    i_shared = (1.0 - w) * self.v1.pid.i + w * self.v0.pid.i
    self.v1.pid.i = i_shared
    self.v0.pid.i = i_shared
    steer = (1.0 - w) * steer_v1 + w * steer_v0
    lateral_output = (1.0 - w) * out_v1 + w * out_v0
    pid_log = log_v0 if w >= 0.5 else log_v1  # the logged state is the controller that owns most of the torque
    if active:
      pid_log.output = float(steer)
    return steer, lateral_output, pid_log
