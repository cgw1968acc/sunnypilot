import numpy as np

interp = np.interp


def apply_low_speed_output_slew(
  output_accel: float,
  last_output_accel: float,
  should_stop: bool,
  v_ego: float,
  a_ego: float,
  max_expected_accel: float,
  allow_fast_release: bool,
  release_lock_active: bool,
) -> float:
  if v_ego >= 1.2:
    return output_accel

  if should_stop and a_ego > (max_expected_accel + 0.02):
    brake_step   = interp(v_ego, [0.0, 0.20, 0.50, 1.20], [0.018, 0.016, 0.013, 0.010])
    release_step = interp(v_ego, [0.0, 0.20, 0.50, 1.20], [0.004, 0.005, 0.007, 0.010])
  elif allow_fast_release:
    brake_step   = interp(v_ego, [0.0, 0.20, 0.50, 1.20], [0.010, 0.009, 0.011, 0.014])
    release_step = interp(v_ego, [0.0, 0.20, 0.50, 1.20], [0.030, 0.026, 0.020, 0.016])
  elif release_lock_active and should_stop:
    brake_step   = interp(v_ego, [0.0, 0.20, 0.50, 1.20], [0.012, 0.010, 0.011, 0.012])
    release_step = interp(v_ego, [0.0, 0.20, 0.50, 1.20], [0.001, 0.0015, 0.0025, 0.005])
  else:
    brake_step   = interp(v_ego, [0.0, 0.20, 0.50, 1.20], [0.010, 0.009, 0.011, 0.014])
    release_step = interp(v_ego, [0.0, 0.20, 0.50, 1.20], [0.003, 0.004, 0.006, 0.010])

  return float(np.clip(output_accel, last_output_accel - brake_step, last_output_accel + release_step))
