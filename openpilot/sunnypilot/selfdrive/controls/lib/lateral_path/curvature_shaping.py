"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants

# Lateral jerk cap on the model's desired curvature. The MACROSTIFF model's path corrections felt a bit stiff to the
# driver (route 53, 2026-09-26). Measured above 60 km/h outside lane changes on 2026-09-25/26: |d(desired curvature)/dt|
# * v^2 has p90 0.70, p95 1.22, p99 2.19 m/s^3. Capping at 1.0 m/s^3 touches ~7% of frames - only the sharpest
# corrections, which then reach the same curvature about 0.1-0.2 s later - and leaves steady centering untouched.
# Bypassed during lane changes (controlsd has its own start-rate cap there) and when lateral control is inactive.


# Lateral jerk cap on the model's desired curvature (ported from tncr18). Softens the sharpest high-speed path
# corrections ("return to centre") while leaving steady centering untouched: capping |d(desired curvature)/dt| * v^2
# at 1.0 m/s^3 only touches the fastest few percent of frames, which then reach the same curvature about 0.1-0.2 s later.
# Bypassed during lane changes (controlsd has its own start-rate cap there) and when lateral control is inactive.
DESIRED_CURVATURE_MAX_LAT_JERK = 1.0  # m/s^3
DESIRED_CURVATURE_JERK_MIN_SPEED = 8.0  # m/s; below this the cap in curvature terms is so loose it barely acts
# Speed schedule (Altis 2026-09-27 21:14, route bf seg 11): after a torque-saturated tight turn at 41-47 km/h the model's
# desired curvature swung -22 -> +6 -> -4 e-3 (~6 m/s^3) and the car swayed twice; a 2.5 m/s^3 cap rounds that off while
# a normal turn-in (~1 m/s^3) is untouched. Off below 30 km/h (tight intersection turns need the full rate), 2.5 from
# 40 to 70 km/h, then the highway 1.0 from 80 km/h.
DESIRED_CURVATURE_JERK_FADE_BP = [8.3, 11.1, 19.4, 22.2]  # m/s (30, 40, 70, 80 km/h)
DESIRED_CURVATURE_JERK_FADE_V = [8.0, 2.5, 2.5, DESIRED_CURVATURE_MAX_LAT_JERK]


class DesiredCurvatureJerkLimiter:
  def __init__(self, dt: float):
    self.dt = dt
    self.prev = 0.0

  def update(self, desired_curvature: float, v_ego: float, active: bool, lane_changing: bool) -> float:
    if not active or lane_changing:
      self.prev = desired_curvature
      return desired_curvature
    max_jerk = float(np.interp(v_ego, DESIRED_CURVATURE_JERK_FADE_BP, DESIRED_CURVATURE_JERK_FADE_V))
    step = max_jerk / max(v_ego, DESIRED_CURVATURE_JERK_MIN_SPEED) ** 2 * self.dt
    self.prev = float(np.clip(desired_curvature, self.prev - step, self.prev + step))
    return self.prev


# Highway path smoothing (driver 2026-10-06: "tnpb2 corrects the wheel poorly at highway speed, a little side to side").
# On every C3X highway straight (routes db/df/ea tnpb1, f2/100 tnpb2) the 2-8 s side-to-side swing of the MODEL's desired
# lateral accel was the same, 0.065-0.080 m/s^2, and the steering followed it in proportion: the swing comes from the
# model's path, not from the torque gains (tncr18's KP differs only below 54 km/h). A first-order filter on the desired
# curvature above 80 km/h takes out the faster part of that swing (2-4 s: -20..-45% with 0.5 s) and leaves slower
# motion and steady curves alone; a highway curve is entered ~0.5 s later. Fades in from 70 km/h like the centering;
# bypassed during lane changes and when lateral control is inactive.
# DISABLED 2026-10-07 (tau 0): its first highway drive on the torque controller, route 0000010f 22:36 at 79 km/h, wove
# side to side ("clearly swaying left and right at 80 km/h"). The model's desired lateral accel swung +-0.8 m/s^2 with a
# 3.3 s period (steering angle std 1.1-2.1 deg per 10 s) against +-0.2 and 0.4-0.6 deg on route 00000100 at the same
# speed, same Macrostiff model, same v1 controller, same corrections except this filter. The feedforward lagged the
# model by ~0.2 s at ~75% amplitude, exactly what a 0.45 s first-order filter does to a 3.3 s swing: the model steers
# from the car's position, so the lag sits inside a closed loop and loses ~40 deg of phase there - it amplified the
# swing it was meant to smooth. (It was designed on logs where the corrections never ran: V0 was active.)
HIGHWAY_SMOOTH_BP = [19.4, 22.2]  # m/s (70, 80 km/h)
HIGHWAY_SMOOTH_TAU_V = [0.0, 0.0]  # s (was [0.0, 0.5])


class HighwayCurvatureSmoother:
  def __init__(self, dt: float):
    self.dt = dt
    self.x = 0.0

  def update(self, desired_curvature: float, v_ego: float, active: bool, lane_changing: bool) -> float:
    tau = float(np.interp(v_ego, HIGHWAY_SMOOTH_BP, HIGHWAY_SMOOTH_TAU_V))
    if not active or lane_changing or tau <= 0.0:
      self.x = desired_curvature
      return desired_curvature
    alpha = self.dt / (tau + self.dt)
    self.x += alpha * (desired_curvature - self.x)
    return self.x


# Turn-exit counter-swing limit (driver 2026-10-06: "at 30 km/h the wheel still sways when straightening out of a turn;
# out of 40 km/h curves too"). Route 00000100 19:00:04 and 20:00:33: the controller tracked its setpoint within ~0.15 s,
# but the MODEL's own path swung past straight after the turn (-2.05 -> +0.33 -> -0.16 m/s^2; -0.14 -> +0.27 -> -0.18)
# and the wheel followed it (125 deg -> -22 deg). The model keeps the lane; what it overdoes is the re-centering just
# after the exit. So below ~50 km/h, for EXIT_WINDOW_T after a real turn (|model lat accel| > EXIT_TURN_MIN) has
# unwound, a request to the OTHER side is capped at EXIT_COUNTER_MAX; the cap is released smoothly at the end of the
# window. A real S-bend is left alone: when the model's own plan 1-3 s ahead asks for more than EXIT_SBEND_MIN to the
# other side, nothing is capped. Off above 55 km/h, during lane changes and when lateral control is inactive.
EXIT_TURN_MIN = 1.0  # m/s^2 of model lateral accel that counts as a turn
EXIT_UNWOUND = 0.3  # m/s^2: the turn has unwound below this
EXIT_WINDOW_T = [0.0, 2.0, 3.0]  # s since unwinding
EXIT_COUNTER_CAP_V = [0.15, 0.15, 2.0]  # m/s^2 allowed to the other side (released over the last second)
EXIT_SPEED_BP = [45.0 / 3.6, 55.0 / 3.6]  # m/s: full below 45 km/h, off above 55
EXIT_SBEND_MIN = 0.5  # m/s^2 to the other side in the model's plan 1-3 s ahead = a real S-bend
EXIT_SBEND_T = (1.0, 1.5, 2.0, 2.5, 3.0)  # s ahead
EXIT_MIN_SPEED = 3.0  # m/s


class TurnExitSwingLimiter:
  def __init__(self, dt: float):
    self.dt = dt
    self.reset()

  def reset(self) -> None:
    self.turn_sign = 0.0  # sign of the turn being exited, 0 when none
    self.in_turn = False
    self.t_exit: float | None = None

  def update(self, desired_curvature: float, model_curvature: float, v_ego: float, active: bool, lane_changing: bool,
             model_v2=None) -> float:
    if not active or lane_changing or v_ego < EXIT_MIN_SPEED or v_ego > EXIT_SPEED_BP[-1]:
      self.reset()
      return desired_curvature
    v2 = v_ego ** 2
    model_lat = model_curvature * v2
    if abs(model_lat) > EXIT_TURN_MIN:
      self.in_turn, self.turn_sign, self.t_exit = True, float(np.sign(model_lat)), None
    elif self.in_turn and abs(model_lat) < EXIT_UNWOUND:
      self.in_turn, self.t_exit = False, 0.0
    if self.t_exit is None:
      return desired_curvature
    self.t_exit += self.dt
    if self.t_exit > EXIT_WINDOW_T[-1]:
      self.reset()
      return desired_curvature
    if model_v2 is not None and len(model_v2.acceleration.y) == len(ModelConstants.T_IDXS):
      ahead = np.interp(EXIT_SBEND_T, ModelConstants.T_IDXS, model_v2.acceleration.y)
      if np.max(-self.turn_sign * ahead) > EXIT_SBEND_MIN:  # a real bend the other way
        self.reset()
        return desired_curvature
    lat = desired_curvature * v2
    if lat * self.turn_sign >= 0.0:
      return desired_curvature
    cap = float(np.interp(self.t_exit, EXIT_WINDOW_T, EXIT_COUNTER_CAP_V))
    fade = float(np.interp(v_ego, EXIT_SPEED_BP, [1.0, 0.0]))
    cap = cap + (abs(lat) - cap) * (1.0 - fade) if abs(lat) > cap else abs(lat)
    return -self.turn_sign * min(abs(lat), cap) / v2
