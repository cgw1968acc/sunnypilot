# Full-chain stop simulation (real long MPC + stop gap governor + LongControl + 0.3 s PCM lag + hybrid creep), scored
# against the driver's perfect-stop template (a = v / 0.65 s below the knee). The compiled MPC is aarch64-only: run it ON
# THE DEVICE, offroad:  /usr/local/venv/bin/python <this file> [CONST=value,...]  (stop_gap / END_TAPER_* overrides)
# The brake onset shaper runs between LongControl and the plant at 33 Hz like the carcontroller. Extra overrides:
# ONSET_FAST=1|2 (onset schedules), RELEASE_J=<m/s^3> (brake release rate).
import sys, types, numpy as np
sys.path.insert(0, "/data/openpilot")
from openpilot.cereal import log, custom
from opendbc.car.structs import car
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, T_IDXS, get_T_FOLLOW, get_safe_obstacle_distance, get_stopped_equivalence_factor
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap import StopGapGovernor, end_decel, STOP_GAP
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
import opendbc.sunnypilot.car.toyota.brake_onset as bo
AGG = log.LongitudinalPersonality.aggressive
DT_MDL, DT_CTRL = 0.05, 0.01
RELEASE_J = 4.0
LAG = 0.3        # s, calibrated from 27 real stops (delivered ~= request 0.25-0.35 s later)
CREEP = 0.06     # m/s^2 hybrid creep when the delivered brake is lighter
class Lead:
  def __init__(s, d, v, a, present=True): s.dRel, s.vLead, s.aLeadK, s.aLeadTau, s.status, s.present, s.modelProb, s.aRel = d, v, a, 1.5, present, present, 1.0, 0.0
class RS:
  def __init__(s, lead): s.leadOne = lead; s.leadTwo = Lead(200, 30, 0, False)

def run(v0, lead_x0, lead_v0, lead_decel, lead_start_t=0.0, see_dist=250.0, secs=40):
  mpc = LongitudinalMpc(dt=DT_MDL); gov = StopGapGovernor(DT_MDL)
  CP = car.CarParams.new_message(stopAccel=-1.0); CP.longitudinalTuning.kiBP=[0.]; CP.longitudinalTuning.kiV=[0.]
  loc = LongControl(CP, custom.CarParamsSP.new_message())
  shaper = bo.BrakeOnsetShaper(0.03, 4.0); cmd = 0.0; first_cmd_t = None
  v, a, x = v0, 0.0, 0.0; xl, vl = lead_x0, lead_v0
  a_tgt, stop = 0.0, False; a_out = 0.0; log_=[]
  t = 0.0; k = 0
  while t < secs:
    if k % 5 == 0:   # planner at 20 Hz
      al = -lead_decel if (t >= lead_start_t and vl > 0) else 0.0
      seen = (xl - x) < see_dist
      lead = Lead(xl - x, vl, al, present=seen)
      mpc.set_weights(True, personality=AGG); mpc.set_cur_state(v, a_tgt)
      mpc.update(RS(lead), personality=AGG)
      vt = np.interp(ModelConstants.T_IDXS[:CONTROL_N], T_IDXS, mpc.v_solution); at = np.interp(ModelConstants.T_IDXS[:CONTROL_N], T_IDXS, mpc.a_solution)
      a_mpc = get_accel_from_plan(vt, at, ModelConstants.T_IDXS[:CONTROL_N], action_t=0.05 + DT_MDL)
      stop_mpc = should_stop(v, a_mpc)
      out = gov.update(True, v, seen, xl - x, vl, a_mpc) if seen else None
      if out is None: gov_active=False
      else: a_mpc, stop_mpc = out; gov_active=True
      a_tgt, stop = float(np.clip(a_mpc, -3.5, 2.0)), stop_mpc
    CS = car.CarState.new_message(vEgo=v, aEgo=a, standstill=v < 1e-3)
    a_out = float(loc.update(True, CS, a_tgt, stop, (-3.5, 2.0), has_lead=True))
    if k % 3 == 0:   # carcontroller at 33 Hz: brake onset shaper + stock 4 m/s^3 wind-up
      step = shaper.down_step(a_out, cmd, bypass=False, v_ego=v, urgent=shaper.is_urgent(a_out, False))
      cmd = float(np.clip(a_out, cmd + step, cmd + (RELEASE_J if cmd < 0 else 4.0) * 0.03))
      if first_cmd_t is None and cmd < -0.05: first_cmd_t = t
    delivered = cmd + (CREEP if cmd > -CREEP and v < 2.0 else 0.0)
    a += (delivered - a) * DT_CTRL / LAG
    v = max(v + a * DT_CTRL, 0.0)
    if v == 0.0 and a > 0 and a_out < 0: a = 0.0
    x += v * DT_CTRL
    if k % 5 == 0:
      vl = max(vl + (-lead_decel if (t >= lead_start_t and vl > 0) else 0.0) * DT_MDL, 0.0)
    xl += vl * DT_CTRL
    log_.append((t, v, a, cmd, xl - x, gov_active if k % 5 == 0 else None))
    t += DT_CTRL; k += 1
  return np.array([r[:5] for r in log_])

def metrics(L):
  t, v, a, out, gap = L.T
  kmh = v * 3.6; dec = -a
  stopped = np.where((v < 0.01) & (t > 1))[0]
  if not len(stopped): return None
  i = stopped[0]
  again = bool(((v[i:] > 0.05)).any())
  approach = (kmh[:i] < 40) & (kmh[:i] > 5)
  # template check below 3 km/h: delivered vs v/0.65 (driver's law)
  m3 = (kmh[:i] < 3.0) & (kmh[:i] > 0.3)
  law = np.array([end_decel(x) for x in v[:i][m3]])
  rms = float(np.sqrt(np.mean((dec[:i][m3] - law) ** 2))) if m3.any() else np.nan
  # front-loading: peak decel speed and whether decel rises again below 10 km/h
  low = (kmh[:i] < 10) & (kmh[:i] > 0.3)
  d_low = dec[:i][low]
  rise = float(np.max(np.maximum.accumulate(d_low[::-1])[::-1] - d_low)) if len(d_low) else 0.0   # not used
  late_peak = float(dec[:i][(kmh[:i] < 6) & (kmh[:i] > 1)].max()) if ((kmh[:i] < 6) & (kmh[:i] > 1)).any() else 0.0
  early_peak = float(dec[:i][(kmh[:i] >= 10)].max()) if (kmh[:i] >= 10).any() else 0.0
  # monotonic below 6 km/h? max increase of decel as speed falls
  seq = dec[:i][(kmh[:i] < 6) & (kmh[:i] > 0.3)]
  bump = float(np.max(seq[1:] - np.minimum.accumulate(seq)[:-1])) if len(seq) > 1 else 0.0
  last02 = float(dec[max(0, i - 20):i].mean())
  b = np.where(out < -0.05)[0]
  if len(b):
    b0 = b[0]; pk = dec[:i].max()
    on01 = float(dec[min(len(dec)-1, b0 + 10 + 30)])   # delivered 0.1 s after the first command (+0.3 s lag)
    t90 = float((np.argmax(dec[b0:i] >= 0.9 * pk)) * DT_CTRL)
  else: on01, t90 = 0.0, 0.0
  jerk = np.diff(a) / DT_CTRL
  return dict(gap=gap[i], second=again, peak=float(dec[:i].max()), early=early_peak, late=late_peak, bump=bump, rms3=rms, last=last02,
              d3=float(np.interp(3.0, kmh[:i][::-1], dec[:i][::-1])), d1=float(np.interp(1.0, kmh[:i][::-1], dec[:i][::-1])),
              d05=float(np.interp(0.5, kmh[:i][::-1], dec[:i][::-1])), maxjerk=float(np.abs(jerk[:i]).max()), on01=on01, t90=t90, mingap=float(gap[:i].min()))

import openpilot.sunnypilot.selfdrive.controls.lib.stop_gap.stop_gap as sg
VARIANT = sys.argv[1] if len(sys.argv) > 1 else "current"
if VARIANT != "current":
  import openpilot.selfdrive.controls.lib.longcontrol as lcm
  for kv in [x for x in VARIANT.split(",") if x.startswith("ONSET_FAST") or x.startswith("RELEASE_J")]:
    k, val = kv.split("=")
    if k == "ONSET_FAST" and float(val) == 1:
      bo.ONSET_T1_V = [0.1, 0.1]; bo.ONSET_T3_V = [0.3, 0.3]; bo.ONSET_J_DOWN = [1.0, 1.0, 4.0]
    if k == "ONSET_FAST" and float(val) == 2:
      bo.ONSET_T1_V = [0.1, 0.1]; bo.ONSET_T3_V = [0.5, 0.5]; bo.ONSET_J_DOWN = [0.5, 0.5, 4.0]
    if k == "RELEASE_J": globals()["RELEASE_J"] = float(val)
  for kv in [x for x in VARIANT.split(",") if not (x.startswith("ONSET_FAST") or x.startswith("RELEASE_J"))]:
    k, val = kv.split("=")
    setattr(lcm if k.startswith("END_TAPER") else (bo if (k.startswith("URGENT") or k.startswith("HARD_")) else sg), k, float(val))
print("variant:", VARIANT)
print(f"{'scenario':46s} gap  mingap 2nd  peak  t90  d@0.1s  d1   d0.5 last0.2")
rows=[]
for V in ((20, 30, 40, 48)):
  v0 = V / 3.6
  for label, d in (("stopped lead, nominal", v0**2/(2*1.2)+8), ("stopped lead, tight", v0**2/(2*1.8)+5), ("stopped lead, very late", v0**2/(2*2.6)+4)):
    L = run(v0, d + 1e-3, 0.0, 0.0, see_dist=d + 0.5); r = metrics(L)
    rows.append((f"{V} km/h {label} ({d:.0f} m)", r))
  tf = get_T_FOLLOW(AGG); d = get_safe_obstacle_distance(v0, tf) - get_stopped_equivalence_factor(v0)
  for dec in (1.0, 2.0):
    L = run(v0, d, v0, dec, lead_start_t=2.0); r = metrics(L)
    rows.append((f"{V} km/h following, lead brakes -{dec:.0f} to stop", r))
for name, r in rows:
  if r is None: print(f"{name:46s} did not stop"); continue
  print(f"{name:46s} {r['gap']:4.1f} {r['mingap']:5.1f}  {'YES' if r['second'] else ' no'} {r['peak']:4.2f} {r['t90']:4.2f}  {r['on01']:5.2f}  {r['d1']:4.2f} {r['d05']:4.2f} {r['last']:4.2f}")
