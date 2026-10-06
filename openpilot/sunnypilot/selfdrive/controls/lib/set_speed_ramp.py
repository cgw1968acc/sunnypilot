"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
# SP (Altis, owner 2026-10-06): "+5 should also accelerate smoothly, parabola-shaped", then "do it all" (the
# speed-step-proportional peak and the soft first touch too). Route 00000100 19:28:00, cruising at 31 km/h, +5: the cruise
# target jumped 0 -> +0.67 in 0.5 s and +0.93 at 1 s and the car peaked at +1.18 m/s^2 (a small surge). After a
# set-speed increase the cruise candidate now follows a planned bell-shaped acceleration instead of jumping to the
# profile's maximum:
#   peak  a_pk = min(accel profile max, SET_SPEED_RAMP_GAIN * speed step)  - proportional to the step
#   edges a smooth S (3x^2 - 2x^3) up over TR and the same down at the end, so the push starts and ends with zero jerk
#         (the soft first touch); TR is as long as the step allows (step / a_pk) but no longer than keeps the peak jerk
#         (1.5 a_pk / TR) at SET_SPEED_RAMP_JERK, and at least TR_MIN - so a small peak at highway speed still comes
#         within ~1 s (driver 2026-10-04 wanted + to respond at once); a plateau at a_pk only when the step is large.
# +5 at 31 km/h: 0 -> peak 0.69 m/s^2 at 2 s -> back to 0 at the new speed after 4 s, jerk never above ~0.5 m/s^3
# (was ~1.0 m/s^2 within 1 s). Only the cruise candidate is shaped; the lead MPC, e2e and the lead start assist are
# untouched. Restarted from the current acceleration on every further press; ends at the new speed, on a set-speed
# decrease, or when not engaged.
SET_SPEED_RAMP_GAIN = 0.5  # m/s^2 of peak per m/s of set-speed step
SET_SPEED_RAMP_JERK = 0.5  # m/s^3 peak jerk the edges aim for
SET_SPEED_RAMP_TR_MIN = 0.8  # s, shortest rise / fall edge
SET_SPEED_RAMP_MIN_STEP = 0.3  # m/s


def _s(x: float) -> float:
  x = min(max(x, 0.0), 1.0)
  return x * x * (3.0 - 2.0 * x)


class SetSpeedRamp:
  def __init__(self):
    self.t = None  # s into the profile, None when inactive

  def start(self, v_ego: float, v_cruise: float, a_now: float, a_max: float) -> None:
    step = v_cruise - v_ego
    self.t = None
    if step < SET_SPEED_RAMP_MIN_STEP or a_max <= 0.05:
      return
    self.a_pk = max(min(a_max, SET_SPEED_RAMP_GAIN * step), 0.05)
    if a_now >= self.a_pk:
      return  # already accelerating harder: the normal cruise law carries on
    self.tr = min(step / self.a_pk, max(SET_SPEED_RAMP_TR_MIN, 1.5 * self.a_pk / SET_SPEED_RAMP_JERK))
    self.tp = max(0.0, (step - self.a_pk * self.tr) / self.a_pk)
    # start on the rising edge where it already equals the current acceleration (no step on a repeated press)
    lo, hi, target = 0.0, 1.0, max(a_now, 0.0) / self.a_pk
    for _ in range(30):
      mid = (lo + hi) / 2.0
      lo, hi = (mid, hi) if _s(mid) < target else (lo, mid)
    self.t = lo * self.tr

  def update(self, dt: float, v_ego: float, v_cruise: float) -> float | None:
    if self.t is None:
      return None
    self.t += dt
    tr, tp = self.tr, self.tp
    if self.t >= 2.0 * tr + tp or v_ego >= v_cruise:
      self.t = None
      return None
    if self.t < tr:
      return self.a_pk * _s(self.t / tr)
    if self.t < tr + tp:
      return self.a_pk
    return self.a_pk * (1.0 - _s((self.t - tr - tp) / tr))

  def cancel(self) -> None:
    self.t = None
