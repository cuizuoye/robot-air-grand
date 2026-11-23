import math
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import compact_vision as c
from trs_vicon_receive_bench import allowed


def test_counter_rate_is_strict_single_instance_diagnostic_not_gap_proof():
    header='\x1b[KTOPIC NAME INST #SUB RATE #Q SIZE\n'
    row='\x1b[Kvehicle_visual_odometry 0 1 32 1 196\n'
    result=c.publication_counter_rate(header+row)
    assert result['rounded_publication_rate_hz']==32
    assert not result['maximum_gap_verified'] and not result['flight_authorized']
    assert c.publication_counter_rate(row) is None
    assert c.publication_counter_rate(header+row+row) is None
    assert c.publication_counter_rate(header+row.replace('0 1 32','1 1 32')) is None


def ground_inputs():
    sent=[dict(elapsed_s=i/32,source_stamp_ns=i+1,position=[1,2,-3],q=[1,0,0,0]) for i in range(33)]
    rows=[]
    for at in (.75,.875,1.):
        stamp=10_000_000+int(at*1e6)
        rows.append(dict(at=at,requested_at=at-.05,requested_boot_us=stamp-50_000,
            completed_boot_us=stamp+1000,snapshot=dict(timestamp=stamp,timestamp_sample=stamp-1,
                pose_frame=1,reset_counter=0,position=[1,2,-3],q=[1,0,0,0])))
    return rows,sent


def test_compact_ground_receipts_are_not_fc_rate_clock_or_flight_evidence():
    rows,sent=ground_inputs();result=c.ground_receipt_evidence(rows,sent)
    assert result['recent_matching_receipts_verified'] and result['host_send_rate_hz']==32
    for key in ('maximum_fc_publication_gap_verified','all_packet_reception_rate_verified',
                'source_clock_synchronization_verified','flight_authorized'):
        assert result[key] is False


@pytest.mark.parametrize('fault',['cached','future','repeated','wrong_pose','old_send','host_gap','host_slow','nonfinite','too_few','bad_receive_time','clock_backwards'])
def test_compact_ground_rejects_bad_or_unmatched_receipts(fault):
    rows,sent=ground_inputs()
    if fault=='cached': rows[0]['snapshot']['timestamp']-=300_000;rows[0]['snapshot']['timestamp_sample']-=300_000
    if fault=='future': rows[0]['snapshot']['timestamp']+=2000;rows[0]['snapshot']['timestamp_sample']+=2000
    if fault=='repeated': rows[1]['snapshot']=dict(rows[0]['snapshot'])
    if fault=='wrong_pose': rows[0]['snapshot']['position']=[9,2,-3]
    if fault=='old_send': sent=sent[:12]
    if fault=='host_gap': sent.pop(16)
    if fault=='host_slow':
        for s in sent:s['elapsed_s']*=1.1
    if fault=='nonfinite': rows[0]['at']=math.nan
    if fault=='too_few': rows.pop()
    if fault=='bad_receive_time': rows[0]['snapshot']['timestamp_sample']-=1001
    if fault=='clock_backwards': rows[0]['completed_boot_us']=rows[0]['requested_boot_us']-1
    with pytest.raises(ValueError):c.ground_receipt_evidence(rows,sent)


def test_explicit_fc_yaw_alignment_rotation_and_roundtrip():
    source=[math.cos(.3),0,0,math.sin(.3)]
    fc=[math.cos(-.2),0,0,math.sin(-.2)]
    angle=c.yaw_alignment(source,fc)
    assert angle==pytest.approx(-1)
    msg,p,q=c.message(m.MAVLink(None),10000,[1,0,-1],source,angle)
    assert p==pytest.approx([math.cos(-1),math.sin(-1),-1])
    assert c.euler(q)==pytest.approx([0,0,-.4])
    assert c.pose_allowed(msg)
    packet=msg.pack(m.MAVLink(None))
    assert len(packet)==128
    decoded=m.MAVLink(None).parse_char(packet)
    assert decoded.get_type()=='VISION_POSITION_ESTIMATE'
    assert decoded.yaw==pytest.approx(-.4,abs=1e-6)
    assert all(decoded.covariance[i]>0 for i in (0,6,11,15,18,20))


@pytest.mark.parametrize('pattern',['missed_slot','29hz','bounded_pause'])
def test_native_receipts_accept_bounded_scheduling_jitter_without_changing_bench(pattern):
    rows,sent=ground_inputs()
    if pattern=='missed_slot':sent.pop(16)  # 62.5ms, not a source/receipt failure.
    elif pattern=='29hz':
        for s in sent:s['elapsed_s']*=1.1
    else:sent=[s for s in sent if not .25<s['elapsed_s']<.625]
    with pytest.raises(ValueError,match=r'rate=.*max_gap='):
        c.ground_receipt_evidence(rows,sent)
    result=c.ground_receipt_evidence(rows,sent,native_rc=True)
    assert result['host_send_rate_diagnostic_only']
    assert not result['host_nominal_cadence_met']
    assert result['recent_matching_receipts_verified'] and not result['flight_authorized']


def test_native_receipts_reject_gap_beyond_existing_mission_budget():
    rows,sent=ground_inputs()
    sent=[s for s in sent if not .25<s['elapsed_s']<.6875]
    with pytest.raises(ValueError,match=r'max_gap=437.500ms; gap_limit=400ms'):
        c.ground_receipt_evidence(rows,sent,native_rc=True)


@pytest.mark.parametrize('fault',['cached','future','mismatch','regression'])
def test_native_receipt_policy_does_not_relax_measurement_validity(fault):
    rows,sent=ground_inputs()
    if fault=='cached':rows[0]['snapshot']['timestamp']-=300000
    elif fault=='future':rows[0]['snapshot']['timestamp_sample']=rows[0]['snapshot']['timestamp']+1
    elif fault=='mismatch':rows[0]['snapshot']['position']=[9,2,-3]
    else:sent[10]['elapsed_s']=sent[9]['elapsed_s']
    with pytest.raises(ValueError):c.ground_receipt_evidence(rows,sent,native_rc=True)


def test_unknown_alignment_tilt_conflict_and_singularity_rejected():
    with pytest.raises(ValueError):
        c.message(m.MAVLink(None),1,[0,0,0],[1,0,0,0],math.nan)
    with pytest.raises(ValueError):
        c.yaw_alignment([math.cos(.1),math.sin(.1),0,0],[1,0,0,0])
    with pytest.raises(ValueError):
        c.euler([math.sqrt(.5),0,math.sqrt(.5),0])


@pytest.mark.parametrize('difference',[4.984427066,5.042756600,5.105368103])
def test_recorded_tilt_residual_uses_explicit_bounded_allowance(difference):
    angle=math.radians(difference)
    source=[math.cos(angle/2),math.sin(angle/2),0.,0.]
    original=list(source)
    assert c.yaw_alignment(source,[1.,0.,0.,0.],tilt_tolerance_deg=6.)==pytest.approx(0.)
    assert source==original
    if difference>5.:
        with pytest.raises(ValueError,match='limit=5.000'):
            c.yaw_alignment(source,[1.,0.,0.,0.])


@pytest.mark.parametrize('invalid',[True,0.,-1.,6.1,math.nan,math.inf])
def test_alignment_allowance_cannot_disable_tilt_check(invalid):
    with pytest.raises(ValueError):
        c.yaw_alignment([1.,0.,0.,0.],[1.,0.,0.,0.],tilt_tolerance_deg=invalid)


def test_compact_transmission_is_an_explicit_trs_receive_only_exception():
    enc=m.MAVLink(None)
    msg,_,_=c.message(enc,10,[1,2,3],[1,0,0,0],0)
    assert allowed(msg,route='trs',safe=True,bench_confirmed=True,compact_timing=True)
    for key in ('safe','bench_confirmed','compact_timing'):
        gates=dict(safe=True,bench_confirmed=True,compact_timing=True)
        gates[key]=False
        assert not allowed(msg,route='trs',**gates)
    assert not allowed(msg,route='usb',safe=True,bench_confirmed=True,compact_timing=True)
    for command in (400,185,176,246,511):
        assert not allowed(enc.command_long_encode(1,1,command,0,1,0,0,0,0,0,0),
            route='trs',safe=True,bench_confirmed=True,compact_timing=True)
    msg.covariance[0]=0
    assert not allowed(msg,route='trs',safe=True,bench_confirmed=True,compact_timing=True)


def row_and_snapshot():
    row={'pc_sent_us':1_050_000,'wire_time_usec':1_040_000,'source_stamp_ns':123,
         'position':[1,2,-3],'q':[1,0,0,0]}
    snap={'pose_frame':1,'reset_counter':0,'position':[1,2,-3],'q':[1,0,0,0],
          'timestamp':70_000,'timestamp_sample':69_999}
    before={'remote_timestamp':1_000_000,'estimated_offset':-1_000_000,
            'observed_offset':-1_000_000,'round_trip_time':1000}
    after={**before,'remote_timestamp':1_200_000}
    return row,snap,before,after


def test_same_source_arrival_measurement_preserves_clock_scope():
    row,snap,before,after=row_and_snapshot()
    measurement=c.arrival_measurement(row,snap,before,after)
    assert measurement['age_at_send_ms']==10
    assert measurement['send_to_fc_publication_ms']==20
    assert measurement['source_ros_publication_to_fc_ms']==30
    assert not measurement['camera_exposure_latency_measured']
    assert not measurement['wireless_clock_conversion_verified']
    assert measurement['fc_publication_minus_sample_us']==1


@pytest.mark.parametrize('problem',['unbracketed','long_rtt','negative_rtt','bad_residual','wrong_frame','wrong_pose','negative_delay'])
def test_invalid_clock_or_pose_does_not_generate_latency_result(problem):
    row,snap,before,after=row_and_snapshot()
    if problem=='unbracketed': after['remote_timestamp']=1_040_000
    if problem=='long_rtt': before['round_trip_time']=10000
    if problem=='negative_rtt': before['round_trip_time']=-1
    if problem=='bad_residual': before['observed_offset']+=3000
    if problem=='wrong_frame': snap['pose_frame']=2
    if problem=='wrong_pose': snap['position']=[2,2,-3]
    if problem=='negative_delay': snap['timestamp']=40000
    with pytest.raises(ValueError): c.arrival_measurement(row,snap,before,after)


def test_fixed_internal_series_query_is_usb_only_and_bounded():
    enc=m.MAVLink(None)
    msg=enc.serial_control_encode(10,2,0,0,len(c.SERIES_COMMAND),c.SERIES_COMMAND.ljust(70,b'\0'))
    assert allowed(msg,route='usb',safe=True,compact_timing=True)
    assert not allowed(msg,route='trs',safe=True,compact_timing=True)
    assert not allowed(msg,route='usb',safe=True)
    assert not allowed(msg,route='usb',compact_timing=True)


def test_transmit_frequency_must_not_replace_measured_internal_series_rate():
    rows=[{'timestamp':1_000_000+i*31_250} for i in range(12)]
    timing=c.series_timing(rows)
    assert timing['rate_hz']==32
    assert timing['maximum_publication_gap_ms']==31.25
    rows[5]['timestamp']=rows[4]['timestamp']
    with pytest.raises(ValueError): c.series_timing(rows)


def test_complete_series_parser_requires_twelve_fresh_blocks():
    block=('TOPIC: vehicle_visual_odometry\n timestamp: {stamp}\n timestamp_sample: {stamp}\n'
           ' pose_frame: 1\n velocity_frame: 0\n reset_counter: 0\n quality: 0\n'
           ' position: [1, 2, -3]\n q: [1, 0, 0, 0]\n')
    text=''.join(block.format(stamp=1_000_000+i*31_250) for i in range(12))
    rows=c.visual_series(text)
    assert c.series_timing(rows)['rate_hz']==32
    assert c.visual_series(block.format(stamp=1)) is None


def test_ground_series_requires_all_48_not_a_short_partial_reply():
    block=('TOPIC: vehicle_visual_odometry\n timestamp: {stamp}\n timestamp_sample: {stamp}\n'
           ' pose_frame: 1\n velocity_frame: 0\n reset_counter: 0\n quality: 0\n'
           ' position: [1, 2, -3]\n q: [1, 0, 0, 0]\n')
    short=''.join(block.format(stamp=1_000_000+i*31_250) for i in range(12))
    full=''.join(block.format(stamp=1_000_000+i*31_250) for i in range(48))
    assert c.visual_series(short, samples=48) is None
    assert c.visual_series(full) is None
    assert c.series_timing(c.visual_series(full, samples=48))['rate_hz']==32
