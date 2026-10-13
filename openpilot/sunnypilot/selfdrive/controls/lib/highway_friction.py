"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

# Highway friction feed-forward scale, ported from tnpb2 (lateral_path/gains.py + latcontrol_torque_ext) on 2026-10-13 at
# the owner's request ("this setting is very good, push it to tncr19 and tnpb2", after driving it on the Altis, tnpb3
# 3eecdcbd44). Altis evidence (route 00000110, open-loop replays): the friction feed-forward acts AGAINST the model-driven
# side-to-side swing on the highway; x1.3 lowered the 0.3-1 Hz torque -7%, 1-3 Hz -8%, torque rate -6% with no added lag,
# 1.3 -> 1.4 another -3% each (owner: "ideal"), and 1.4 -> 1.5 by 100 km/h was rated "very good" on 2026-10-13.
# Applied to the stock (v1) torque controller only, on top of whatever friction torqued / the override sets.
HWY_FRICTION_BP = [60.0 / 3.6, 70.0 / 3.6, 100.0 / 3.6]  # m/s
HWY_FRICTION_SCALE_V = [1.0, 1.4, 1.5]


def get_highway_friction_scale(v_ego: float) -> float:
  return float(np.interp(v_ego, HWY_FRICTION_BP, HWY_FRICTION_SCALE_V))


class HighwayFrictionScale:
  """Scales torque_params.friction by speed every frame without compounding: the base is re-read whenever someone else
  (torqued, the override) changed the value since our last write; the written value is read back because the capnp
  field is float32."""
  def __init__(self):
    self.base: float | None = None
    self.applied: float | None = None

  def apply(self, torque_params, v_ego: float) -> None:
    if self.applied is None or torque_params.friction != self.applied:
      self.base = torque_params.friction
    torque_params.friction = self.base * get_highway_friction_scale(v_ego)
    self.applied = torque_params.friction
