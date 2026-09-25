# tnpbmod: pristine tn-prebuilt (5c37fd3ced) + the complete tncr18 feature set

Created 2026-09-26. The tree of this branch is byte-identical to tncr18 at 36742d7e96; the 108 incremental
commits of tncr18 have been regrouped into five feature commits on top of the untouched sunnypilot prebuilt base.

| # | Feature group | Main files |
|---|---|---|
| 1 | Corolla Cross Hybrid car port: fingerprint, steer actuator delay 0.6 s, TSS2 longitudinal ki table, heavier aEgo filter, raw cruise stalk bits from 0x361, hybrid engine state (`carStateSP.engineOff`) | `opendbc/car/toyota/*`, `custom.capnp` |
| 2 | Software-owned cruise set speed ("ceiling mode"): short +/- presses move openpilot's target, a long press only moves the PCM's own number, target capped at PCM + 7 km/h, target kept across CANCEL/RES, HUD shows the target on the dashboard's calibration (dial = 1.068 v + 2.73 km/h, road-verified HUD 80 = dial 80) | `cruise.py`, `card.py`, `hud_renderer.py` |
| 3 | Cruise switch mirror experiment + panda TX allowlist + rebuilt panda firmware (dormant unless its params are set; can be dropped as one commit) | `cruise_switch_mirror.py`, `safety/modes/toyota.h`, `panda/board/obj` |
| 4 | Lateral: curve outward bias (deadzone 0.0005 1/m; 0 / 0.09 / 0.155 / 0.19 at 40 / 70 / 90 / 120 km/h), lane centering feedback (PD + slow integral on the lane-line offset, capped at 0.6 m/s^2), lane-change turn-in rate interpolated with speed | `latcontrol_torque.py`, `controlsd.py` |
| 5 | Longitudinal: eco accel profile (60/70/80 km/h breakpoints, EV-only line while the engine is off, 0.85x throttle with no lead), gentle cruise decel to a lowered set speed (eco: speed-scheduled coasting, -0.22 at 70 and -0.30 below 60 km/h), lead start assist, MPC follow-time tweaks, early stopped-vehicle warning (> 70 km/h, TTC < 5 s) | `accel_controller.py`, `longitudinal_planner.py`, `long_mpc.py`, `lead_start_assist/`, `stationary_lead_warning.py`, `events.py` |

## Not included

The tip of `tn-prebuilt` carries 14 manual edits made between 2026-09-16 and 2026-09-20 (stop accel in
`interfaces.py`, carcontroller ki values, `toyotacan` ALLOW_LONG_PRESS, accel_controller, `policy.py`,
`process_config.py`). This branch was built from the pristine base on purpose, so those edits are not here;
most of them have newer equivalents inside the feature commits above.

## Not yet road-tested as of 2026-09-26

Lane centering feedback, eco coasting decel, the 0.85x no-lead throttle, and the stopped-vehicle warning.
