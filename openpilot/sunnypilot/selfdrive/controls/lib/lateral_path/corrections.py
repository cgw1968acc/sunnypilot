"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.curve_bias import apply_curve_outward_bias
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.straight_mode import StraightMode
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.lane_centering import LaneCentering, LaneTrimLowSpeed
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.lane_centering_low_speed import LaneCenteringLowSpeed, \
  LOW_CENTER_ENABLED
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.curvature_shaping import DesiredCurvatureJerkLimiter, \
  HighwayCurvatureSmoother, TurnExitSwingLimiter, PlanAverageSmoother, PLAN_AVG_ENABLED, EXIT_ENABLED
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.low_speed_straight_hold import LowSpeedStraightHold


class LateralPathCorrections:
  """Everything tnpb2 does to the model's desired curvature before the torque controller sees it, in this order:
  highway plan averaging, low-speed straight hold, curve outward bias, highway lane centering, low-speed lane centering (or the older trim),
  turn-exit counter-swing limit, (NNLC path correction hand-off), desired-curvature jerk cap, highway smoothing. See
  each module for the reasons and data."""
  def __init__(self, dt: float):
    self.lane_centering = LaneCentering(dt)
    self.lane_trim = LaneTrimLowSpeed(dt, self.lane_centering)
    self.low_centering = LaneCenteringLowSpeed(dt, self.lane_centering) if LOW_CENTER_ENABLED else None
    self.curvature_jerk_limiter = DesiredCurvatureJerkLimiter(dt)
    self.highway_smoother = HighwayCurvatureSmoother(dt)
    self.turn_exit_limiter = TurnExitSwingLimiter(dt) if EXIT_ENABLED else None
    self.plan_average = PlanAverageSmoother(dt) if PLAN_AVG_ENABLED else None
    self.straight_mode = StraightMode(dt)
    self.low_straight = LowSpeedStraightHold(dt)

  def update(self, desired_curvature: float, CS, active: bool, model_v2, set_path_correction=None) -> float:
    lane_changing = model_v2 is not None and model_v2.meta.laneChangeState != 0
    straight = self.straight_mode.update(model_v2, CS.vEgo, active, lane_changing)  # tncr: fastest corrections on a straight
    if self.plan_average is not None:
      # highway: replace the model action's fast wobble by the average of its own plan ahead (forward-looking)
      desired_curvature = self.plan_average.update(desired_curvature, model_v2, CS.vEgo, active, lane_changing)
    # low speed, surely straight: keep the wheel still instead of following the action's hops around zero
    desired_curvature = self.low_straight.update(desired_curvature, model_v2, CS, active, lane_changing)
    model_curvature = desired_curvature
    desired_curvature = apply_curve_outward_bias(desired_curvature, CS.vEgo)
    desired_curvature += self.lane_centering.update(model_v2, CS.vEgo, active, CS.steeringPressed, straight,
                                                    desired_curvature, lane_changing)
    if self.low_centering is not None:
      desired_curvature += self.low_centering.update(model_v2, CS.vEgo, active, CS.steeringPressed,
                                                     getattr(CS, 'leftBlinker', False) or getattr(CS, 'rightBlinker', False),
                                                     lane_changing)
    else:
      desired_curvature += self.lane_trim.update(model_v2, CS.vEgo, active, CS.steeringPressed, model_curvature)
    if self.turn_exit_limiter is not None:
      desired_curvature = self.turn_exit_limiter.update(desired_curvature, model_curvature, CS.vEgo, active, lane_changing,
                                                        model_v2)
    if set_path_correction is not None:
      # NNLC builds its feedforward from the model's own future lateral accelerations, which carry none of the
      # corrections above; hand it the same correction so its future inputs describe the corrected path
      set_path_correction(model_curvature, desired_curvature, CS.vEgo)
    desired_curvature = self.curvature_jerk_limiter.update(desired_curvature, CS.vEgo, active, lane_changing, straight)
    desired_curvature = self.highway_smoother.update(desired_curvature, CS.vEgo, active, lane_changing)
    return desired_curvature
