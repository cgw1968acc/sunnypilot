# AGENTS.md - rules for every AI / agent session on this branch

Branch `tnpb2`: sunnypilot for a **Toyota Corolla Altis Hybrid** (comma 3X), built on rav4kumar's prebuilt base
`tn-prebuilt` 2f35bb4a19 (sunnypilot v2026.10.02, AGNOS 19.7). The owner road-tests every change and reports in
Traditional Chinese; rav4kumar (sunnypilot maintainer) pulls commits from here. Read this file before changing code.

## 1. Plug-in rule (rav4kumar, 2026-10-07) - MUST

> "Make very minimal changes on the stock controller; make an inherited controller and just plug into stock or
> update it, so stock functions as is, the new feature can be easily turned on and off, and merging with comma
> sync is easy."

- New behaviour goes into a **sunnypilot module** (`openpilot/sunnypilot/...` or `opendbc_repo/opendbc/sunnypilot/...`),
  never as feature code inside a stock file. Put it in the **existing sunnypilot class** for that stock component when
  there is one (rav4kumar): `LatControlTorqueExt` (lateral), `LongitudinalPlannerSP` (planner), `ControlsExt`
  (controlsd), `CarStateExt` (Toyota car state), `fingerprints_ext.py` (Toyota fingerprints), `brake_onset.py` /
  `auto_brake_hold.py` (Toyota long control). Only where no such class exists (long_mpc, radard) add a
  `*_ext.py` module under the sunnypilot directory. Small reusable pieces (`set_speed_ramp.py`, `lead_dropout_hold.py`,
  `wheel_pulse.py`, `lateral_path/`) may stay separate helper modules used by those classes.
- The stock file gets **one hook line** per plug-in point, marked `# sunnypilot hook`, that calls the module and
  returns the stock value unchanged when the feature is off. Method names in those classes end in `_sp`.
- Every feature has a dedicated **Params key**, defined in `openpilot/common/params_keys.h`, persistent and default-on
  in tnpb2 (e.g. `TnLeadDropoutHold`). Set the relevant key to false with `Params.put_bool()` to restore that feature's
  stock behavior. Read it with `get_tn_switch(params, key)` (`openpilot/sunnypilot/__init__.py`, opendbc twin in
  `opendbc/sunnypilot/car/toyota/values.py`): it is `Params.get(..., return_default=True)`, but on a prebuilt whose
  compiled `libparams_c.so` predates the key (UnknownKeyName - the C3X on 2026-10-07) it reads the param file
  `/data/params/d/<key>` directly (missing = on, `0` = off). CAUTION (verified 2026-10-08): on such a prebuilt the
  manager's `params.clear_all()` at boot deletes every param file whose key the compiled library does not know, so
  `printf 0 > /data/params/d/<key>` only lasts until the next reboot. To switch a feature off persistently there, change
  its default in the code; the file route becomes persistent once the prebuilt is rebuilt with the updated
  `params_keys.h`. Do not use JSON override files or another feature registry.
- With every switch off the behaviour must equal stock. Prove it (see 4).
- Car-specific code must check the car: the Altis is `TOYOTA_COROLLA_TSS2` + `ToyotaFlags.HYBRID` + a Corolla-sedan
  ("12" series) firmware part number (`is_altis_hybrid()` in `opendbc/sunnypilot/car/toyota/brake_onset.py`); the
  Corolla Cross shares the platform ("16" series) and must not be affected.
- Fix the layer that owns a behaviour; prove the existing stack cannot do it before adding a module (Kumar,
  2026-10-02: too many layers = never-ending tuning).

## 2. Where things are (plug-in map)

| Area | Module | Hook in stock file |
|---|---|---|
| Lateral path corrections (curve outward bias, highway lane centering, low-speed lane trim, turn-exit counter-swing limit, desired-curvature jerk cap, highway smoothing) | `openpilot/sunnypilot/selfdrive/controls/lib/lateral_path/` (`corrections.py` runs them in order) | `latcontrol_torque.py` -> `self.extension.correct_desired_curvature()` |
| Altis low-speed KP schedule, NNLC only from 30 km/h | `lateral_path/gains.py`, `lateral_path/nnlc_blend.py`, applied in `latcontrol_torque_ext.py` | none (extension object) |
| Planner: set-speed ramp, set-speed throttle-gate override, uphill scaling, highway ease, lead start assist | `*_sp` methods of `LongitudinalPlannerSP` (`openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py`); helpers `set_speed_ramp.py`, `lead_start_assist/` | `longitudinal_planner.py`, 5 hooks |
| MPC desired distance (trim + light-onset margin) | `long_mpc_ext.py` | `long_mpc.py`, 1 hook |
| Radar match filters, lead drop-out hold | `radard_ext.py`, `lead_dropout_hold.py` | `radard.py`, 2 hooks |
| Lane-change start curvature rate | `ControlsExt.lane_change_start_rate_sp()` (`openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py`) | `controlsd.py`, 1 hook |
| Screen speed, wheel-pulse creep | `*_sp` methods of `CarStateExt` (`opendbc/sunnypilot/car/toyota/carstate_ext.py`); helper `wheel_pulse.py` | `carstate.py`, 4 hooks |
| Hybrid detection from CAN | `hybrid_can_messages()` in `opendbc/sunnypilot/car/toyota/fingerprints_ext.py` | `interface.py`, 1 condition |
| Brake onset shaping (+ creep-follow gas release), overshoot limit (+ Altis moderate band, `TnToyotaBrakeOvershootModerate`), Altis regen hand-over feed-forward + low-speed PID hold (`TnToyotaBrakeUnderdelivery`) | `opendbc/sunnypilot/car/toyota/brake_onset.py` | `carcontroller.py` (shaper/corrections construction, `condition_pid` before the PID, `apply` after it) |
| Auto brake hold | `opendbc/sunnypilot/car/toyota/auto_brake_hold.py` | `carcontroller.py` (Kumar's) |
| End of stop / standstill hold; Altis: re-roll hold (`TnStopRerollHold`), flat-ground hold -1.2 (`TnStandstillHoldFlat`), creep-follow stop at -0.24 (`TnCreepFollowStop`) | `openpilot/sunnypilot/selfdrive/controls/lib/stopping_controller.py` | `controlsd.py` (Kumar's; passes `CP` and `pitch`) |

Known stock touch points that are fixes, not features: `selfdrived.py` (micd/qcomgpsd ignored), `system/micd.py`,
`system/qcomgpsd/qcomgpsd.py`, `system/timed.py` (a dead mic/GPS must not block engagement; clock restore),
`toyotacan.py` (`ALLOW_LONG_PRESS` 2 = owner's setting; `hold_decel` argument for the auto hold).

**Lateral controller caveat:** with `EnforceTorqueControl` off, `controlsd_ext.initialize_lateral_control()` uses
sunnypilot's `LatControlTorqueV0`, which has only the curve bias and the jerk cap. NNLC cannot be on together with
`EnforceTorqueControl` or `LateralJerkTorqueController` (the UI turns both off). Check which controller the car runs
before judging a lateral change.

## 3. Commits

Every commit carries its full reasoning **in English in the commit message**, for others to read:
`Why` (the driver's report: date, route + time, what was felt), `Evidence` (numbers from logs / replays), `Change`
(files, constants old -> new), `Scope` (what is untouched, what is not yet road-tested), `Revert`. One-line
messages are not acceptable. Code comments keep the same reasoning next to the constants.

## 4. Verifying

- Host unit tests (Params stubbed; never let a stub point at /tmp):
  `PYTHONPATH=<tree>:<tree>/opendbc_repo python3 <stub runner> <test file>`; opendbc tests with `python3 -m pytest`
  inside `opendbc_repo`. The longitudinal MPC `.so` in this tree is aarch64, so anything importing
  `longitudinal_planner` / `long_mpc` runs on the device only.
- A refactor must be behaviour-identical: replay real segments through the old and the new tree and compare
  outputs (lateral: run `LatControlTorque` over logged frames on the host; radard/plannerd/controlsd/card:
  `selfdrive/test/process_replay` on the device, with `config_realtime_process` disabled because offroad cores are
  offline). Expect max |diff| 0.0.
- Tuning changes: show the computed effect (tables from logs / replays) before committing when the owner is only
  asking or exploring; commit and deploy only on an explicit decision.

## 5. Owner's standing rules

- Never change `T_FOLLOW`.
- Altis brake feel: light, early, continuous; no two-stage stops; end of stop per the owner's end table.
- Altis and Corolla Cross tunes are separate (Cross: branch `tncr19`).
- Answers to the owner in Traditional Chinese.
