# tnpb1 (Corolla Altis Hybrid, comma 3X) - custom feature code map

Base: `tn-prebuilt` at 4f909d0fed (sunnypilot v2026.09.01 prebuilt + the owner's ki/stop-accel edits).
Everything below is what tnpb1 adds on top. Use this as the checklist when porting to another branch.
Paths are relative to the repository root. Sign conventions: openpilot curvature and model y are positive to the
RIGHT in this build; `steeringAngleDeg` is positive to the LEFT.

## 1. Stop-and-hold behind a lead ("autohold" as the owner calls it)

Three pieces work together; port all three.

| Piece | File | What it does |
|---|---|---|
| Stop gap governor | `openpilot/sunnypilot/selfdrive/controls/lib/stop_gap/stop_gap.py` (+ `tests/`) | Below 8.5 m/s with a stopped/crawling lead it replaces the lead MPC target with a profile that ends 3.0 m behind the lead: constant deceleration (0.8-1.6 m/s^2) then a linear glide below 0.8 m/s so v reaches 0 at the point. Only ever adds braking; hands back when the lead pulls away. |
| Firm standstill hold | `opendbc_repo/opendbc/car/toyota/interface.py` | `ret.stopAccel = -1.0` for the hybrid (stock -0.02 lets creep torque roll the car). longcontrol's stopping state ramps to it at 0.3 m/s^2/s (1.0 once standing), so the approach stays soft and the hold is firm. The HYBRID flag is forced for `TOYOTA_COROLLA_TSS2` in `_get_params` because the Altis's hybrid ECU answers the firmware query only about half the time; without it every hybrid-gated tuning (this hold, 0.05 s actuator delay, engine state) silently switched off for that drive. |
| Lead start assist | `openpilot/sunnypilot/selfdrive/controls/lib/lead_start_assist/lead_start_assist.py` (+ `tests/`) | When stopped behind a lead that starts moving, puts an acceleration floor under the target right away instead of waiting for the MPC to close the gap. Arms after both cars are stopped 0.2 s; needs real lead motion (speed and gap growth held 0.25 s). |

Hooks, all in `openpilot/selfdrive/controls/lib/longitudinal_planner.py` (`LongitudinalPlanner`):
- imports of `LeadStartAssist` and `StopGapGovernor`; instances created in `__init__` with `self.dt`; both `.reset()` when long control is off.
- after the MPC/cruise candidates are computed: if `self.mpc.source == LongitudinalPlanSource.lead0`, call
  `self.stop_gap.update(long_allowed, v_ego, lead_one.present, lead_one.dRel, lead_one.vLead, output_a_target_mpc)`
  and, when it returns a value, replace `(output_a_target_mpc, output_should_stop_mpc)` with it; otherwise `reset()`.
- after the final `output_a_target` is chosen: `a_start = self.lead_start_assist.update(assist_allowed, v_ego, present, dRel, vLead, vRel)`;
  if `a_start` is not None and larger than the target, use it and clear `output_should_stop`.
- `long_allowed = not long_control_off and not force_decel and not brakePressed and not gasPressed`;
  `assist_allowed = long_allowed and not (e2e and e2e says stop)`.

## 2. Brake onset shaping (Toyota)

- `opendbc_repo/opendbc/sunnypilot/car/toyota/brake_onset.py` (+ `tests/test_brake_onset.py`): `BrakeOnsetShaper`. Downward jerk
  of the PCM accel command follows 0.25 m/s^3 for 0.2 s, then 0.4 / 0.8 / 1.8 / 4.0 m/s^3 at 0.6 / 1.0 / 1.5 s, restarting on
  every new brake request; requests below -2 m/s^2 and FCW bypass it; release is untouched.
- Hook in `opendbc_repo/opendbc/car/toyota/carcontroller.py`: construct `self.brake_onset = BrakeOnsetShaper(DT_CTRL * 3, -ACCEL_WINDDOWN_LIMIT / (DT_CTRL * 3))`;
  where `pcm_accel_cmd` is rate limited (`if CC.longActive:`), use `winddown_step = self.brake_onset.down_step(pcm_accel_cmd, self.prev_accel, bypass=self.brake_onset.is_urgent(pcm_accel_cmd, fcw_alert))`
  as the negative step of `rate_limit(...)`, and `self.brake_onset.reset()` in the else branch.

## 3. Longitudinal tuning constants (owner's values)

- `opendbc_repo/opendbc/car/toyota/carcontroller.py` `get_long_tune`: `kiBP = [0., 1.0, 3.0, 5.0, 12., 27., 36.]`, `kiV = [0.35, 0.32, 0.280, 0.23, 0.21, 0.10, 0.09]`.
- `openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py` `get_jerk_factor`: relaxed 1.0, standard 1.3, aggressive 0.5.
- `openpilot/sunnypilot/selfdrive/controls/lib/accel_controller/accel_controller.py` `MAX_ACCEL_PROFILES`: eco `[1.85, 1.55, 0.35, 0.13, 0.10]`, normal `[1.95, 1.80, 0.70, 0.36, 0.25]`.
- Note: `COMFORT_BRAKE` / `STOP_DISTANCE` in long_mpc.py are compiled into the acados solver; on a prebuilt branch they cannot be changed
  by editing the constant. A runtime-equivalent is to shift the lead obstacle by `v^2/2 * (1/b_new - 1/2.5)` per horizon stage before it is
  passed as a parameter.

## 4. Lateral: curve outward bias

- `openpilot/selfdrive/controls/lib/latcontrol_torque.py`: `apply_curve_outward_bias(desired_curvature)` relaxes the commanded curvature
  above a 0.003 1/m deadzone by `CURVE_OUTWARD_FRAC = 0.12`; called at the top of `LatControlTorque.update` (+ `openpilot/selfdrive/controls/tests/test_curve_outward_bias.py`).
  (tncr18 carries a newer speed-scheduled version of the same idea plus lane centering; port from there if wanted.)

## 5. Robustness (comma 3X specific)

- `openpilot/selfdrive/selfdrived/selfdrived.py`: `self.ignored_processes = {'mapd', 'micd'}` - a dead mic no longer blocks engagement.
- `openpilot/system/micd.py`: retries the capture stream forever instead of exiting.
- `openpilot/system/timed.py`: saves the last valid time to `/data/last_known_time` every 60 s and restores it at boot (the 3X's RTC does not hold time).
- `openpilot/system/manager/process_config.py`: loggerd and uploader enabled; `openpilot/selfdrive/monitoring/policy.py`: `_PHONE_THRESH = 0.5`.

## Porting order

1. Feature 1 (three pieces + planner hooks), run `stop_gap/tests` and `lead_start_assist/tests`.
2. Feature 2 (brake onset + carcontroller hook), run `test_brake_onset.py`.
3. Constants (3), then 4 and 5 as needed.
