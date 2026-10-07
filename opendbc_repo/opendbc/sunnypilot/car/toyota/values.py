"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class ToyotaFlagsSP(IntFlag):
  SMART_DSU = 1
  RADAR_CAN_FILTER = 2
  ZSS = 4
  STOCK_LONGITUDINAL = 8
  STOP_AND_GO_HACK = 16
  SP_ENHANCED_BSM = 32
  SP_NEED_DEBUG_BSM = 64
  SP_AUTO_BRAKE_HOLD = 128
  TSS2_LONG_TUNING = 512


class ToyotaSafetyFlagsSP:
  DEFAULT = 0
  UNSUPPORTED_DSU = 1
  GAS_INTERCEPTOR = 2


def get_tn_switch(params, key: str) -> bool:
  """tnpb2 feature switch, default on. This tree is prebuilt: libparams_c.so only knows the keys it was compiled
  with, so a key added to params_keys.h after the build raises UnknownKeyName (seen on the C3X, 2026-10-07:
  Params().get("TnPlannerExtensions") -> UnknownKeyName). Then the param file is read directly - missing = on,
  "0" = off - so the switches work before and after the next prebuilt build includes the keys."""
  if params is None:
    return True
  try:
    return bool(params.get(key, return_default=True))
  except Exception:  # UnknownKeyName on a prebuilt without the key
    try:
      with open(params.get_param_path(key)) as f:
        return f.read().strip() != "0"
    except OSError:
      return True
