"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NeuralNetworkLateralControl
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import LatControlTorqueExtOverride
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.corrections import LateralPathCorrections
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.gains import KP_INTERP_TN, get_highway_friction_scale
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_path.nnlc_blend import get_nnlc_weight


class LatControlTorqueExt(NeuralNetworkLateralControl, LatControlTorqueExtOverride):
  def __init__(self, lac_torque, CP, CP_SP, CI):
    NeuralNetworkLateralControl.__init__(self, lac_torque, CP, CP_SP, CI)
    LatControlTorqueExtOverride.__init__(self, CP)

    # tnpb2 additions apply to the stock torque controller (latcontrol_torque.LatControlTorque) only, as before the
    # plug-in refactor; sunnypilot's LatControlTorqueV0 (used when EnforceTorqueControl is off) creates this extension too
    from openpilot.selfdrive.controls.lib.latcontrol_torque import INTERP_SPEEDS, LatControlTorque
    self.tn_stock_controller = isinstance(lac_torque, LatControlTorque)
    # tnpb2: Altis low-speed KP schedule on the stock torque PID (same speed breakpoints as stock)
    if self.tn_stock_controller:
      lac_torque.pid._k_p = [INTERP_SPEEDS, KP_INTERP_TN]
    # tnpb2: corrections to the model's desired curvature (lateral_path/), applied via correct_desired_curvature()
    self.path_corrections = LateralPathCorrections(lac_torque.dt)
    self.nnlc_low_speed_handover = self.tn_stock_controller
    # tnpb2: highway friction feed-forward scale (lateral_path/gains.py), applied in update_override_torque_params()
    self.hwy_friction = self.tn_stock_controller
    self._fric_base: float | None = None
    self._fric_applied: float | None = None
    self._v_ego = 0.0

  def update_override_torque_params(self, torque_params) -> bool:
    """sunnypilot hook at the start of LatControlTorque.update(): the override first, then the tnpb2 highway friction
    scale on top of whatever friction is current (torqued / override). Stock = unchanged."""
    changed = LatControlTorqueExtOverride.update_override_torque_params(self, torque_params)
    if not self.hwy_friction:
      return changed
    if self._fric_applied is None or torque_params.friction != self._fric_applied:
      self._fric_base = torque_params.friction  # set by torqued or the override since our last scaling
    torque_params.friction = self._fric_base * get_highway_friction_scale(self._v_ego)
    self._fric_applied = torque_params.friction  # read back: the capnp field is float32
    return changed

  def correct_desired_curvature(self, desired_curvature: float, CS, active: bool) -> float:
    """Hook called by LatControlTorque.update() before it uses the desired curvature; stock = unchanged."""
    self._v_ego = CS.vEgo
    return self.path_corrections.update(desired_curvature, CS, active, self.model_v2, self.set_path_correction)

  def update(self, CS, VM, pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
             desired_lateral_accel, actual_lateral_accel, lateral_accel_deadzone, gravity_adjusted_lateral_accel,
             desired_curvature, actual_curvature, steer_limited_by_safety, output_torque):
    # tnpb2: with NNLC on, the low-speed range stays with the torque controller (see lateral_path/nnlc_blend.py)
    nnlc_weight = get_nnlc_weight(CS.vEgo) if (self.nnlc_low_speed_handover and self._nnlc_enabled) else 1.0
    if nnlc_weight <= 0.0:
      return pid_log, output_torque
    torque_output = output_torque
    pid_log, output_torque = self._update(CS, VM, pid, params, ff, pid_log, setpoint, measurement, calibrated_pose,
                                          roll_compensation, desired_lateral_accel, actual_lateral_accel,
                                          lateral_accel_deadzone, gravity_adjusted_lateral_accel, desired_curvature,
                                          actual_curvature, steer_limited_by_safety, output_torque)
    if nnlc_weight < 1.0:
      output_torque = nnlc_weight * output_torque + (1.0 - nnlc_weight) * torque_output
    return pid_log, output_torque

  def _update(self, CS, VM, pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
              desired_lateral_accel, actual_lateral_accel, lateral_accel_deadzone, gravity_adjusted_lateral_accel,
              desired_curvature, actual_curvature, steer_limited_by_safety, output_torque):
    self._ff = ff
    self._pid = pid
    self._pid_log = pid_log
    self._setpoint = setpoint
    self._measurement = measurement
    self._roll_compensation = roll_compensation
    self._lateral_accel_deadzone = lateral_accel_deadzone
    self._desired_lateral_accel = desired_lateral_accel
    self._actual_lateral_accel = actual_lateral_accel
    self._desired_curvature = desired_curvature
    self._actual_curvature = actual_curvature
    self._gravity_adjusted_lateral_accel = gravity_adjusted_lateral_accel
    self._steer_limited_by_safety = steer_limited_by_safety
    self._output_torque = output_torque

    self.update_calculations(CS, VM, desired_lateral_accel)
    self.update_jerk_aware_torque_control(CS, roll_compensation, gravity_adjusted_lateral_accel)
    self.update_neural_network_feedforward(CS, params, calibrated_pose)

    return self._pid_log, self._output_torque
