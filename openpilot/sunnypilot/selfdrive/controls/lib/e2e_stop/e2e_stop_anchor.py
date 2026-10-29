"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

# Experimental-mode stop with no lead: land where the model FIRST said, not where its stop point drifts back to.
# tncr route 000000be 16:58:48-16:59:01 (owner 2026-10-29, bookmark: "stopped far before the line"): at 30-50 m the
# model's stop point (where its planned speed drops below STOP_V) sat on the stop line (odometer ~762-764 m). Its own
# action.desiredAcceleration then braked harder than that point needs (-0.64..-0.83 vs v^2/2d = -0.53), the car slowed
# sooner, the model - which predicts from the car's current motion - moved its stop point closer, asked for still more,
# and so on: 762 -> 758 -> 755.5 m. The car stopped at 755.7 m, ~6 m before the line.
#
# Dynamic, per model frame, only while the e2e candidate is the one in control and no radar lead is near the stop:
#  - the stop point is fixed on the ground (odometer) when the model commits; it follows the model forward at once;
#  - a retreat beyond the model's frame-to-frame jitter (RETREAT_DB) is treated as the self-fulfilling drift above: the
#    model's own accel is scaled by (its distance / the anchored distance less RETREAT_DB), which keeps its shape (firm
#    middle, soft end) but lands RETREAT_DB before the anchored point, never softer than the constant decel that still
#    lands there (v^2/2d) and never firmer than the model; the scale is exactly 1 at the jitter limit (no step);
#  - a sudden retreat (more than RETREAT_JUMP in one frame) or a firm model request (below URGENT_A) is new information
#    (something stepped in): the anchor jumps to the model's point and the model is followed as it is;
#  - while it acts, the e2e stop request (should_stop) is held off; once the landing point is within STOP_D, or within
#    the distance a soft end (A_END) needs from the current speed, the model's own accel and stop request take over
#    again, so the end of the stop is the model's (and the end layers') as always and has room before the point;
#    a car that has (nearly) stopped is never pulled off again, wherever the anchored point is.
# When the model's stop point does not drift back, the output is the model's own accel, unchanged.
STOP_V = 0.3          # m/s: model trajectory speed counted as "stopped"
RETREAT_DB = 2.0      # m: model stop point jitter on an ordinary approach; also the landing margin before the anchored point
RETREAT_JUMP = 4.0    # m: retreat in one model frame = new obstacle, follow the model
URGENT_A = -1.5       # m/s^2: model braking at least this firmly is followed as it is
STOP_D = 1.0          # m: e2e stop request allowed within this of the landing point
STANDSTILL_V = 0.3    # m/s: once (nearly) stopped, never pull the car off again: the model's stop as it is
A_END = 0.5           # m/s^2: soft-end decel the end of the stop (model + end layers) is given room for
LEAD_CLEAR = 8.0      # m: a radar lead nearer than the anchored point + this hands the stop to the lead logic


def model_stop_x(positions, velocities) -> float | None:
  for x, v in zip(positions, velocities, strict=False):
    if v < STOP_V:
      return float(x)
  return None


class E2eStopAnchor:
  def __init__(self):
    self.odo = 0.0
    self.stop_at = None   # anchored stop point, odometer m
    self.last_model_at = None
    self.active = False

  def reset(self):
    self.stop_at = None
    self.last_model_at = None
    self.active = False

  def update(self, enabled: bool, v_ego: float, dt: float, stop_x: float | None, a_model: float,
             lead_d: float | None) -> tuple[float, bool] | None:
    """enabled: e2e candidate in control, longitudinal active, no driver input. stop_x: model stop distance (m) or None.
    Returns (accel, allow_stop_request) or None when it does not act."""
    self.odo += max(v_ego, 0.0) * dt
    if not enabled or stop_x is None:
      self.reset()
      return None
    model_at = self.odo + stop_x
    if lead_d is not None and lead_d < stop_x + LEAD_CLEAR:
      self.reset()
      return None

    sudden = self.last_model_at is not None and self.last_model_at - model_at > RETREAT_JUMP
    if self.stop_at is None or model_at > self.stop_at or sudden or a_model < URGENT_A:
      self.stop_at = model_at
    self.last_model_at = model_at

    d_anchor = self.stop_at - self.odo
    d_land = d_anchor - RETREAT_DB
    if stop_x >= d_land:
      self.active = False
      return a_model, True

    # hand back early enough that the soft end of the stop (A_END from the current speed) still ends before the point,
    # at the landing point, or once stopped: the model's own end and stop request
    if d_land <= max(STOP_D, v_ego ** 2 / (2.0 * A_END)) or v_ego < STANDSTILL_V:
      self.active = False
      return a_model, True

    self.active = True
    need = v_ego ** 2 / (2.0 * d_land)
    scaled = a_model * stop_x / d_land
    accel = float(np.clip(scaled, a_model, -need)) if a_model <= -need else a_model
    return accel, False
