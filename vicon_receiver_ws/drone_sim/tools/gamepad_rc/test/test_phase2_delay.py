from copy import deepcopy
import math

import pytest

from ev_delay_calibration import calibrate
from test_phase2_acceptance import no_grant


def capture(lag=37):
    def value(t):return [math.sin(7*t)+.7*math.sin(17*t),.5*math.cos(11*t),.4*math.sin(23*t)]
    return dict(alignment=dict(clock_domain='fc_boot_ns',timestamp_mode='fc_receive_time',
        vision_timestamp_field='vehicle_visual_odometry.timestamp_sample',
        imu_timestamp_field='vehicle_imu.timestamp_sample',body_frame='FRD',
        clock_uncertainty_ns=1_000_000,observation_matching_evidence='SYNTHETIC',
        rigid_body_revision_sha256='a'*64,orientation_transform_evidence='SYNTHETIC',
        clock_mapping_evidence='SYNTHETIC'),
        imu=[dict(t_ns=1_000_000_000+i*10_000_000,rate_body_rad_s=value(i*.01)) for i in range(600)],
        vision=[dict(t_ns=1_000_000_000+i*10_000_000+lag*1_000_000,rate_body_rad_s=value(i*.01)) for i in range(600)])


def test_effective_delay_recovered_on_synthetic_holdout_not_65ms_approval():
    r=calibrate(capture())
    assert r['candidate_identified'],r
    assert abs(r['effective_delay_candidate_ms']-37)<=1
    assert not r['parameter_approved'] and not r['component_delays_identified']
    no_grant(r)


@pytest.mark.parametrize('fault',['static','no_fc_stamp','no_clock_mapping','wrong_frame','clock_uncertain',
                                  'short','nonmonotonic','nonfinite'])
def test_insufficient_delay_evidence_rejected(fault):
    c=capture()
    if fault=='static':
        for rows in (c['imu'],c['vision']):
            for row in rows:row['rate_body_rad_s']=[0.,0.,0.]
    elif fault=='no_fc_stamp':c['alignment']['vision_timestamp_field']='pc_arrival_ns'
    elif fault=='no_clock_mapping':del c['alignment']['clock_mapping_evidence']
    elif fault=='wrong_frame':c['alignment']['body_frame']='rigid_body_unaligned'
    elif fault=='clock_uncertain':c['alignment']['clock_uncertainty_ns']=50_000_000
    elif fault=='short':c['imu']=c['imu'][:50]
    elif fault=='nonmonotonic':c['vision'][2]['t_ns']=c['vision'][1]['t_ns']
    else:c['imu'][2]['rate_body_rad_s'][0]=math.nan
    r=calibrate(c)
    assert not r['candidate_identified'] and not r['parameter_approved']
    no_grant(r)
