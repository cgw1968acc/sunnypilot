from openpilot.sunnypilot.selfdrive.controls.lib.lead_dropout_hold import LeadDropoutHold, DROPOUT_HOLD_T

DT = 0.05


def radar_lead(d, v_lead, a, v_ego):
  return {'present': True, 'radar': True, 'dRel': d, 'vLead': v_lead, 'vLeadK': v_lead, 'aLeadK': a,
          'vRel': v_lead - v_ego, 'yRel': 0.0, 'aLeadTau': 1.5, 'modelProb': 0.9, 'radarTrackId': 7}


def vision_lead(d, v_lead, v_ego):
  return {'present': True, 'radar': False, 'dRel': d, 'vLead': v_lead, 'vLeadK': v_lead, 'aLeadK': 0.0,
          'vRel': v_lead - v_ego, 'yRel': 0.0, 'aLeadTau': 0.3, 'modelProb': 0.9, 'radarTrackId': -1}


def test_23_55_47_dropout_keeps_the_braking_radar_lead():
  # route 0000010d: radar lead 36 m braking at -1.2 while we close at 6 m/s, then a vision-only lead "at 49 m"
  h = LeadDropoutHold(DT)
  v_ego = 13.8
  h.update(radar_lead(36.0, 12.1, -1.2, v_ego), v_ego)
  out = h.update(vision_lead(49.3, 12.9, v_ego), v_ego)
  assert out['radar'] and out['dRel'] < 36.0 and out['aLeadK'] < -1.0     # still the near, braking lead
  # the radar track comes back: it is taken at once
  back = radar_lead(out['dRel'] - 0.5, 11.0, -2.2, v_ego)
  assert h.update(back, v_ego) is back
  # a second short drop-out to a vision target only 3.5 m farther is held as well (23:55:50.0)
  assert h.update(vision_lead(back['dRel'] + 3.5, 12.0, v_ego), v_ego)['radar']


def test_vision_lead_that_agrees_is_used():
  h = LeadDropoutHold(DT)
  v_ego = 13.8
  h.update(radar_lead(36.0, 12.1, -1.2, v_ego), v_ego)
  near = vision_lead(36.5, 12.0, v_ego)
  assert h.update(near, v_ego) is near


def test_hold_ends_after_one_second():
  h = LeadDropoutHold(DT)
  v_ego = 13.8
  h.update(radar_lead(36.0, 12.1, -1.2, v_ego), v_ego)
  far = vision_lead(49.3, 12.9, v_ego)
  n = 0
  while h.update(far, v_ego) is not far:
    n += 1
    assert n * DT <= DROPOUT_HOLD_T + DT
  assert n * DT >= DROPOUT_HOLD_T - DT


def test_steady_or_receding_lead_is_not_held():
  h = LeadDropoutHold(DT)
  v_ego = 13.8
  h.update(radar_lead(36.0, 12.0, 0.0, v_ego), v_ego)        # closing on it but it is not braking
  far = vision_lead(49.3, 14.5, v_ego)
  assert h.update(far, v_ego) is far


def test_lead_changing_lanes_away_ends_the_hold_after_a_second_at_most():
  h = LeadDropoutHold(DT)
  v_ego = 13.8
  h.update(radar_lead(30.0, 12.0, -1.0, v_ego), v_ego)
  gone = {'present': False}
  outs = [h.update(gone, v_ego) for _ in range(int(DROPOUT_HOLD_T / DT) + 3)]
  assert outs[0]['present'] and outs[-1] is gone


def test_a_closer_new_lead_is_taken_at_once():
  h = LeadDropoutHold(DT)
  v_ego = 13.8
  h.update(radar_lead(36.0, 12.1, -1.2, v_ego), v_ego)
  h.update(vision_lead(49.3, 12.9, v_ego), v_ego)           # hold starts
  cut_in = radar_lead(15.0, 12.0, 0.0, v_ego)
  assert h.update(cut_in, v_ego) is cut_in


def test_far_braking_lead_is_not_held():
  h = LeadDropoutHold(DT)
  v_ego = 25.0
  h.update(radar_lead(80.0, 20.0, -1.0, v_ego), v_ego)
  far = vision_lead(95.0, 20.0, v_ego)
  assert h.update(far, v_ego) is far
