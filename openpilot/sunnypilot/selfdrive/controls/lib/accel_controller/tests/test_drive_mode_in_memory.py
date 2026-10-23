from openpilot.sunnypilot.selfdrive.controls.lib.accel_controller.accel_controller import AccelController, AccelProfile


def make():
  ac = AccelController.__new__(AccelController)  # no Params on the host
  ac._profile = AccelProfile.normal
  return ac


def test_can_drive_mode_overrides_a_stale_param():
  # route 000000a5 21:30: the param file could not be written during a storage stall; the profile from carStateSP wins
  ac = make()
  ac.set_profile_sp(AccelProfile.sport)
  assert ac.profile == AccelProfile.sport
  ac.set_profile_sp(AccelProfile.eco)
  assert ac.profile == AccelProfile.eco


def test_out_of_range_profile_is_ignored():
  ac = make()
  before = ac.profile
  ac.set_profile_sp(7)
  assert ac.profile == before


def test_carstate_ext_publishes_the_profile():
  from opendbc.car import structs
  from opendbc.sunnypilot.car.toyota import carstate_ext
  src = open(carstate_ext.__file__).read()
  assert 'ret_sp.driveModeProfile = int(self._prev_accel_profile)' in src
  sp = structs.CarStateSP()
  assert sp.driveModeValid is False and sp.driveModeProfile == 0
