"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

# NNLC only from 30 km/h up (owner 2026-10-07, option B). Route 0000010d (2026-10-06 night, first drive with NNLC
# actually on): at 16-30 km/h the NNLC torque output flipped between +1.0 and -1.0 every ~0.25 s (00:04:02 left turn
# into a lane, 23:57:57 turn exit) while the measured lateral accel tracked the setpoint within ~0.3 m/s^2 - felt as
# side-to-side shaking and a sudden swerve when entering the lane. Torque control at the same places (route 00000100
# 19:00, NNLC off) gave a smooth +0.1..+0.3. Cause: NNLC runs its PID on THIS controller's PID object
# (extension.update gets self.pid), whose gain schedule KP_INTERP (6.0 at 5 m/s, 3.2 at 7.5, 2.0 at 10) was tuned for
# lateral-accel units; in NNLC's torque units the same gains are several times stronger, and NNLC adds its own
# low-speed curvature term on top. From ~40 km/h KP is down to ~0.8 and NNLC was calm. Below NNLC_BLEND_V_BP[0] the
# extension is not run at all (so it also no longer updates the shared PID there); between the breakpoints the two
# outputs are blended. No effect when NNLC is off.
NNLC_BLEND_V_BP = [30.0 / 3.6, 45.0 / 3.6]  # m/s: torque control below, NNLC above, linear in between


def get_nnlc_weight(v_ego: float) -> float:
  return float(np.interp(v_ego, NNLC_BLEND_V_BP, [0.0, 1.0]))
