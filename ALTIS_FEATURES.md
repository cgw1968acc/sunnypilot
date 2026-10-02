# tnpb2 (Corolla Altis Hybrid, comma 3X) - custom feature code map

Base: `tn-prebuilt` at 2f35bb4a19 (sunnypilot v2026.10.02-4917, CI build of rav4kumar's be85cb61; AGNOS 19.7).
tnpb2 = that base + every tnpb1 feature EXCEPT the smooth-stop work (2026-10-02, owner's decision: "restart tuning
smooth stopping from here"). Longitudinal stopping is therefore Kumar's: stock `longcontrol.py`, his
`sunnypilot/selfdrive/controls/lib/stopping_controller.py` hooked in `controlsd.py`, `stopAccel` left at the opendbc
default (the `-0.02` line is commented out), stock `long_mpc.py`.

## Carried over from tnpb1 (ported 2026-10-02)

| Area | File(s) | What it does |
|---|---|---|
| Hybrid detection from CAN | `opendbc_repo/opendbc/car/toyota/interface.py` `hybrid_can_messages()` (+ `tests/test_hybrid_detection.py`) | HYBRID flag set when 0x127 + 0x245 are broadcast and 0x2C1 is not; the firmware query alone missed the hybrid ECU on ~half the boots. |
| Wheel pulse creep | `opendbc_repo/opendbc/sunnypilot/car/toyota/wheel_pulse.py`, hook in `carstate.py` (SPEED 0xB4 ENCODER) | Below the ~0.5 km/h wheel-speed floor the encoder still counts, so a creeping car is not reported as standstill. |
| Speedometer-matched screen speed | `carstate.py` `cluster_speed()` (1.05 v + 1.47 km/h above 5 km/h, 0.1 km/h hysteresis) | On-screen speed equals the dash. |
| Auto brake hold | `opendbc_repo/opendbc/sunnypilot/car/toyota/auto_brake_hold.py` (+ tests) | Engages 1 s after a firm press (>= 1400 N) or 2.5 s after a real stop while holding >= 900 N; latched once engaged. Needs `ToyotaAutoHold=1`. |
| Brake / engage onset shaping | `brake_onset.py` + `carcontroller.py` hook | Already in Kumar's base (his port of the tnpb1 shaper); unchanged here. The tnpb1 regen->friction `handover_scale` is NOT carried over. |
| `ALLOW_LONG_PRESS` = 2 | `opendbc_repo/opendbc/car/toyota/toyotacan.py` | Owner's ACC_CONTROL setting. |
| Lead start assist | `openpilot/sunnypilot/selfdrive/controls/lib/lead_start_assist/` + hook in `longitudinal_planner.py` | Acceleration floor as soon as a stopped lead moves off (owner: confirmed good). |
| Highway set-speed ease | `longitudinal_planner.py` `V_CRUISE_EASE_MIN` / `J_CRUISE_EASE_DOWN` | Above 90 km/h a lowered set speed eases the throttle off at 0.2 m/s^3. |
| Accel profiles (owner's) + eco | `openpilot/sunnypilot/selfdrive/controls/lib/accel_controller/accel_controller.py` | Owner's eco/normal/sport tables, engine-off eco line, speed-scheduled eco cruise decel with coasting, eco lead pull-away boost (`test_eco_lead_boost.py`). The planner passes `radarState.leadOne` for the boost. |
| Radar side-object rejection | `openpilot/selfdrive/controls/radard.py` (+ `tests/test_radard_side_object.py`) | A stationary radar point far to the side of a clearly moving camera lead is not matched. |
| Lateral | `openpilot/selfdrive/controls/lib/latcontrol_torque.py`, `sunnypilot/.../latcontrol_torque_v0.py` (+ tests) | Lane centering feedback, low-speed lane trim, curve outward bias, jerk-based rate limits (ported from tncr18). |
| Lane-change start rate | `openpilot/selfdrive/controls/controlsd.py` `_lane_change_start_curv_rate()` | Speed-dependent cap on how fast the curvature builds in the first seconds of a lane change. |
| Robustness | `selfdrived.py` (`micd`, `qcomgpsd` ignored), `system/micd.py`, `system/timed.py`, `system/qcomgpsd/qcomgpsd.py` | A dead mic or GPS no longer blocks engagement; last known time saved/restored. |

## Added on tnpb2 (end of stop, 2026-10-02 night)

`openpilot/sunnypilot/selfdrive/controls/lib/stopping_controller.py` (Kumar's module, hooked in `controlsd.py`) holds
the request at `END_REQUEST` (-0.15, ~660 N) from `END_V` (3 km/h) until the wheels stop whenever the plan is braking
to a stop (`a_target <= END_PLAN_MIN`, latched until the plan wants to go): never lighter (creep) and never firmer
(nod); a plan at or below `END_HARD` (-1.5) passes through. From the driver's 42 manual stops (manual_stops.py,
lightest_stop.py): the pedal stays constant to the stop; 640 N is the lightest force that stopped and held, 680 N+
held every time, 600 N and lighter crept; the PCM's settled force below 3 km/h is ~480 + 1200 x |request| N. After
standstill the stock hold delay and ramp to `stopAccel` apply unchanged (with the default -2.0 that is ~2900 N over
~4 s; tnpb1 used -1.0 = ~1840 N). Tests: `openpilot/sunnypilot/selfdrive/controls/tests/test_stopping_controller.py`.

## Deliberately NOT carried over (smooth-stop work, restart from Kumar's base)

- `openpilot/sunnypilot/selfdrive/controls/lib/stop_gap/` (governor, device campaign, tests) and its planner hooks / A-B flag.
- `openpilot/sunnypilot/selfdrive/controls/lib/lead_need_cap/` and its planner hook.
- All tnpb1 edits to `openpilot/selfdrive/controls/lib/longcontrol.py` (end taper, emergency floor, stopping-state
  easing, standstill hold) and `controlsd.py`'s `stop_decel_floor` / `radarState` wiring.
- `long_mpc.py` edits (jerk factors, low-speed braking-lead tau).
- `interface.py` `stopAccel` values (-1.0 hybrid / -2.0 petrol) and `carcontroller.py` `handover_scale`.

## Validation
Host (pure python, params stubbed): `PYTHONPATH=/home/marc8/sunnypilot-tnpb2:/home/marc8/sunnypilot-tnpb2/opendbc_repo
/home/marc8/sunnypilot/.venv/bin/python /home/marc8/claude_work/run_stubbed2.py <test files>`. Tests that import the
planner / MPC need the device (acados). Kumar's `test_longcontrol.py` does not collect on his base (imports a removed
`STOPPING_SPEED`); not touched here.
