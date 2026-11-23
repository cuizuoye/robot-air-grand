"""Real fixed topic parsers + MAVLink framing; all inputs synthetic/offline."""
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mission_audit_collector as p
from mavlink_crc_stream import CrcResyncParser
from test_mission_live_inputs import audit


def reply_text(key, now, *, final_change=False):
    if key=='critical_status':
        blocks=[reply_text(k,now).split('TOPIC:',1)[1].rsplit('nsh>',1)[0]
                for k in ('local_position','commander')]
        return 'nsh> '+p.COMMANDS[key].decode()+''.join('TOPIC:'+b for b in blocks)+'nsh> '
    stamp=int((now+10)*1e6)
    row,commander=audit(now,0)
    if key=='actuator':
        values=dict(timestamp=stamp,armed=False,lockdown=True,manual_lockdown=False,force_failsafe=False)
    elif key=='selector':
        values={**row['selector'], 'accel_device_id':1,'gyro_device_id':2,'last_instance_change':1,
                'baro_device_id':3,'mag_device_id':0,'combined_test_ratio':[0.]*9,'relative_test_ratio':[0.]*9}
        if final_change:values['instance_changed_count']+=1
    elif key=='commander':values=commander
    elif key=='local_position':
        values={**row['local'], 'timestamp_sample':stamp-100,'x':1.,'y':2.,'z':-.1,'heading':0.,
                **dict.fromkeys(('vx','vy','vz','eph','epv','evh','evv'),0.)}
    elif key=='failsafe':values=row['failsafe']
    elif key.startswith(('flags_','status_')):
        values=row[key.split('_')[0]]
        if key.startswith('flags_'):
            values={**values,'timestamp_sample':stamp-100,
                    **dict.fromkeys(('cs_rng_hgt','cs_gps_hgt','cs_fake_hgt','reject_optflow_y'),False)}
    else:
        prefix,slot=key.rsplit('_',1)
        values={**row[prefix],'estimator_instance':1-int(slot)}  # Deliberately swapped uORB slots.
        for field in ('observation','observation_variance','innovation','innovation_variance','test_ratio'):
            values[field]=[.01,.01] if prefix=='aid' else .01
    topic=p.COMMANDS[key].decode().split()[1]
    def format_value(value):
        if isinstance(value,list):return '['+', '.join(str(x) for x in value)+']'
        return str(value)
    return ('nsh> '+p.COMMANDS[key].decode()+'TOPIC: '+topic+'\n'
            +''.join(f' {name}: {format_value(value)}\n' for name,value in values.items())+'\nnsh> ')


def deliver(collector,key,now,*,corrupt=False,final_change=False):
    fc=m.MAVLink(None,srcSystem=1,srcComponent=1)
    parser=CrcResyncParser()
    text=reply_text(key,now,final_change=final_change).encode()
    result=None
    for offset in range(0,len(text),70):
        part=text[offset:offset+70]
        wire=bytearray(fc.serial_control_encode(10,0,0,0,len(part),part.ljust(70,b'\0')).pack(fc))
        fc.seq=(fc.seq+1)%256
        if corrupt and offset==70:wire[-1]^=1
        for msg in parser.parse_buffer(wire):
            if msg.get_type()=='BAD_DATA':collector.damaged_packet(now)
            elif msg.get_type()=='SERIAL_CONTROL':
                result=collector.receive(bytes(msg.data[:msg.count]).decode(),now,int((now+10)*1000)) or result
    return result


def test_combined_critical_read_is_one_bounded_command_with_isolated_topics():
    text=reply_text('critical_status',1)
    assert len(p.COMMANDS['critical_status'])<=70
    assert p.allowed(p.message(m.MAVLink(None),'critical_status'))
    parsed=p.parse('critical_status',text)
    assert parsed['local']['timestamp']==parsed['commander']['timestamp']==11_000_000
    # The second topic must never fill a missing timestamp in the first.
    assert p.parse('critical_status',text.replace(' timestamp: 11000000\n','',1)) is None
    assert p.parse('critical_status',text.split('TOPIC: vehicle_status')[0]+'nsh> ') is None


@pytest.mark.parametrize('corrupt',[False,True])
def test_critical_reply_is_not_a_full_audit_and_crc_damage_cannot_refresh_it(corrupt):
    from rc_watch import NativeRCMonitor
    c=p.MissionAuditCollector(native_rc=NativeRCMonitor())
    c.key='critical_status';c.requested_at=1
    c.transaction.begin(p.COMMANDS[c.key],1)
    deliver(c,'critical_status',1.01,corrupt=corrupt)
    assert not c.take_audits() and c.last_snapshot is None
    assert bool(c.status_updates) is not corrupt
    assert bool(c.critical_samples) is not corrupt
    if not corrupt:
        assert set(c.status_updates[0]['values'])=={'local','commander'}


def drive(collector, *, cycles=1, step=.02):
    now=0.;keys=[];rows=[]
    for _ in range(250):
        action=collector.action(now,True)
        if action and action!='poll':
            keys.append(action)
            deliver(collector,action,now+.001)
            rows.extend(collector.take_audits())
        now+=step
        if len(rows)>=cycles:break
    return rows,keys,now


def test_full_parser_pipeline_produces_real_complete_rows_with_swapped_aid_slots():
    collector=p.MissionAuditCollector()
    rows,keys,_=drive(collector,cycles=3)
    assert len(rows)==3 and not collector.disabled
    assert collector.mapping==dict(aid=0,height_aid=0,yaw_aid=0)
    assert all(r['commander']['pre_flight_checks_pass'] for r in rows)
    assert all(r['row']['local']['xy_reset_counter']==0 for r in rows)
    assert all(r['row']['aid']['estimator_instance']==1 for r in rows)
    assert all(r['row']['yaw_comparison'] is None for r in rows)  # No invented aligned yaw proof.
    assert keys.count('aid_1')==1  # Discover once, recheck explicit ownership each new row.
    assert keys.count('selector')==6 and keys.count('actuator')>=6


def test_failed_visual_read_drains_to_prompt_then_only_refreshes_ground_state():
    c=p.MissionAuditCollector()
    c.key='visual_series';c.requested_at=0.;c.attempts=1
    c.transaction.begin(p.COMMANDS[c.key],0.)
    c.receive('nsh> '+p.COMMANDS[c.key].decode()+'TOPIC: vehicle_visual_odometry\n',.1,10100)
    assert c.action(.2,True,restoration_only=True)=='poll'
    assert c.draining and 'vehicle_visual_odometry' in c.failures[-1]['reply_tail']
    assert c.action(.3,True,restoration_only=True) is None
    c.receive('timeout waiting for update\nnsh> ',2.2,12200)
    assert c.action(2.21,True,restoration_only=True)=='actuator'
    assert deliver(c,'actuator',2.22)
    assert not c.receive_groups and not c.take_audits()
    assert c.action(2.23,True,restoration_only=True)=='actuator'
    assert not c.disabled


def test_cancelled_visual_without_prompt_does_not_overlap_or_forge_ground_state():
    c=p.MissionAuditCollector()
    c.key='visual_series';c.requested_at=0.;c.attempts=1
    c.transaction.begin(p.COMMANDS[c.key],0.)
    assert c.action(.1,True,restoration_only=True)=='poll'
    assert c.action(3.11,True,restoration_only=True) is None
    assert c.disabled and c.last_snapshot is None


def test_audit_start_is_not_critical_local_sampling_time():
    collector=p.MissionAuditCollector()
    rows,_,_=drive(collector,step=.05)
    assert len(rows)==1
    assert .5 < rows[0]['now']-rows[0]['requested_at'] <= 2
    assert rows[0]['now']-(rows[0]['row']['local']['timestamp']/1e6-10) < .5


def test_slow_full_audit_is_not_mislabeled_fresh():
    c=p.MissionAuditCollector();rows,_,_=drive(c,step=.2)
    assert not rows and c.rejected_rows
    assert c.rejected_rows[0]['duration_s']>2


@pytest.mark.parametrize('native',[False,True])
@pytest.mark.parametrize('duration',[5.361059464,8.001])
def test_complete_slow_row_uses_native_budget_without_restamping(native,duration):
    from copy import deepcopy
    from rc_watch import NativeRCMonitor
    c=p.MissionAuditCollector()
    parsed,_,_=drive(c)
    record=parsed[0]
    c.native_rc=NativeRCMonitor() if native else None
    c.row={**deepcopy(record['row']),'commander':deepcopy(record['commander'])}
    c.row_started=record['requested_at']
    now=c.row_started+duration
    c._complete(now)
    result=c.take_audits()
    if native and duration<8.:
        assert len(result)==1
        assert result[0]['row']==record['row']
        assert result[0]['requested_at']==record['requested_at']
        assert result[0]['now']==now
        assert not c.rejected_rows
    else:
        assert not result
        assert c.rejected_rows[-1]['reason']=='full_diagnostic_row_expired'
    assert not c.row and c.row_started is None


def test_bad_frame_isolates_epoch_drains_and_retries_exact_same_read():
    c=p.MissionAuditCollector();assert c.action(0,True)=='actuator'
    deliver(c,'actuator',.01,corrupt=True)
    assert c.last_snapshot is None
    assert c.action(.02,True)=='actuator'
    snapshot=deliver(c,'actuator',.03)
    assert snapshot and c.live_lockdown(.03) is True
    assert c.failures and c.reads==1


def test_missing_prompt_blocks_instead_of_overlapping_or_retrying_another_topic():
    c=p.MissionAuditCollector();assert c.action(0,True)=='actuator'
    c.receive('nsh> listener actuator_armed -n 1\npartial',.1,10100)
    assert c.action(3.1,True)=='poll'
    assert c.action(6.2,True) is None
    assert c.disabled and c.fault


def test_selector_change_inside_row_cannot_publish_audit():
    c=p.MissionAuditCollector();now=0.;selector_count=0
    for _ in range(100):
        key=c.action(now,True)
        if key and key!='poll':
            if key=='selector':selector_count+=1
            deliver(c,key,now+.001,final_change=key=='selector' and selector_count>1)
        now+=.02
        if c.disabled:break
    assert c.disabled and not c.take_audits()


def test_unsafe_actuator_raises_but_no_control_command_is_generated():
    c=p.MissionAuditCollector();c.action(0,True)
    text=reply_text('actuator',.01).replace('armed: False','armed: True')
    with pytest.raises(RuntimeError,match='Unsafe'):
        c.receive(text,.01,10010)
    assert c.disabled


def test_default_monitor_gate_still_denies_new_fixed_queries_and_every_control_write():
    import trs_runtime_monitor as monitor
    e=m.MAVLink(None)
    for key in ('commander','selector','flags_1','aid_0','local_position'):
        msg=p.message(e,key)
        assert not monitor.allowed(msg,identity_fresh=True)
        assert monitor.allowed(msg,identity_fresh=True,audit_shadow=True)
    for command in (176,400,21,22,246,511):
        msg=e.command_long_encode(1,1,command,0,1,0,0,0,0,0,0)
        assert not monitor.allowed(msg,identity_fresh=True,audit_shadow=True)
    assert not monitor.allowed(e.param_set_encode(1,1,b'EKF2_EV_CTRL',11,6),
                               identity_fresh=True,audit_shadow=True)


def test_actuator_tolerance_switch_cannot_change_new_audit_transaction_policy():
    from trs_runtime_monitor import query_readiness_fault
    c=p.MissionAuditCollector();c.action(0,True);deliver(c,'actuator',.01,corrupt=True)
    assert query_readiness_fault(c,False) is query_readiness_fault(c,True) is None
    c.action(.02,True);deliver(c,'actuator',.03,corrupt=True);c.action(.04,True)
    assert c.disabled
    assert query_readiness_fault(c,False)==query_readiness_fault(c,True)
    assert query_readiness_fault(c,True) is not None


def test_never_published_aid_is_missing_fusion_not_broken_link_or_fake_readiness():
    c=p.MissionAuditCollector();now=0.
    for _ in range(100):
        key=c.action(now,True)
        if key and key!='poll':
            if key.startswith(('aid_','height_aid_','yaw_aid_')):
                text='nsh> '+p.COMMANDS[key].decode()+'never published\nnsh> '
                c.receive(text,now+.001,int((now+10)*1000))
            else:deliver(c,key,now+.001)
        now+=.02
    assert not c.disabled and not c.failures and not c.take_audits()
    assert c.rejected_rows and c.unavailable_aid_slots
    assert c.last_snapshot['at']>1.5  # Critical actuator reads keep progressing.


def test_old_aid_payload_is_never_refreshed_but_does_not_disable_actuator_diagnostics():
    c=p.MissionAuditCollector();now=0.
    for _ in range(70):
        key=c.action(now,True)
        if key and key!='poll':
            if key.startswith(('aid_','height_aid_','yaw_aid_')):
                c.receive(reply_text(key,now-5),now+.001,int((now+10)*1000))
            else:deliver(c,key,now+.001)
        now+=.02
    assert not c.disabled and not c.take_audits()
    assert all(row['reason']=='cached_aid_not_current' for row in c.unavailable_aid_slots.values())


def test_same_two_hz_actuator_sample_is_not_new_proof_or_a_transport_fault():
    c=p.MissionAuditCollector();c.action(0,True);deliver(c,'actuator',.01)
    original=c.last_snapshot.copy()
    assert c.action(.02,True)=='selector';deliver(c,'selector',.03)
    c.key=None;c.queue.clear();c.queue.append('actuator')
    assert c.action(.1,True)=='actuator'
    text=reply_text('actuator',.01)
    assert c.receive(text,.11,10110) is None
    assert c.last_snapshot==original and not c.disabled and not c.failures
    assert c.noncurrent_actuator_reads[-1]['reason']=='repeated'
    assert c.live_lockdown(.7) is None


def test_cached_actuator_reply_does_not_refresh_current_lock():
    c=p.MissionAuditCollector();c.action(0,True)
    assert c.receive(reply_text('actuator',0),.7,10700) is None
    assert c.last_snapshot is None and c.live_lockdown(.7) is None
    assert not c.disabled and not c.failures


@pytest.mark.parametrize('delay',[.6,.8,1.0])
def test_one_second_ground_profile_accepts_original_age_but_not_strict_flight_age(delay):
    c=p.MissionAuditCollector(ground_status_max_age=1.)
    c.action(0,True)
    snapshot=c.receive(reply_text('actuator',0),delay,int(10000+delay*1000))
    assert snapshot and snapshot['timestamp_us']==10_000_000
    assert snapshot['fc_age_at_receive_s']==pytest.approx(delay)
    assert c.live_lockdown(delay) is True
    assert c.live_lockdown(delay,max_age=.5) is None
    assert c.live_lockdown(1.001) is None


def test_one_second_ground_profile_still_rejects_overage_and_future_samples():
    c=p.MissionAuditCollector(ground_status_max_age=1.);c.action(0,True)
    assert c.receive(reply_text('actuator',0),1.001,11001) is None
    assert c.last_snapshot is None
    assert c.noncurrent_actuator_reads[-1]['maximum_age_s']==1.
    c=p.MissionAuditCollector(ground_status_max_age=1.);c.action(0,True)
    assert c.receive(reply_text('actuator',1),.1,10100) is None
    assert c.last_snapshot is None and c.failures


def test_ground_tolerance_never_refreshes_duplicates_or_ignores_unsafe_cached_state():
    c=p.MissionAuditCollector(ground_status_max_age=1.);c.action(0,True)
    original=c.receive(reply_text('actuator',0),.6,10600).copy()
    c.queue.clear();c.queue.append('actuator');c.action(.7,True)
    assert c.receive(reply_text('actuator',0),.8,10800) is None
    assert c.last_snapshot==original and c.live_lockdown(1.001) is None
    c=p.MissionAuditCollector(ground_status_max_age=1.);c.action(0,True)
    with pytest.raises(RuntimeError,match='Unsafe'):
        c.receive(reply_text('actuator',0).replace('armed: False','armed: True'),.8,10800)
    assert c.disabled


@pytest.mark.parametrize('profile',[.5,1.])
@pytest.mark.parametrize('ahead_us',[1,500,25_000,50_000])
def test_usb_reply_ahead_of_rf_clock_is_not_current_proof(profile,ahead_us):
    c=p.MissionAuditCollector(ground_status_max_age=profile);c.action(0,True)
    owner_us=10_000_000-ahead_us
    assert c.receive(reply_text('actuator',0),.01,owner_us//1000,boot_us=owner_us) is None
    assert c.last_snapshot is None and c.live_lockdown(.01) is None
    assert not c.failures and not c.disabled and c.transaction.idle
    assert c.noncurrent_actuator_reads[-1]['reason']=='ahead_of_owner_clock'
    assert c.action(.05,True) is None
    assert c.action(.12,True)=='actuator'
    # The actual FC source stamp may be read again, but is only accepted
    # once the owner's original clock proves it is no longer future.
    snap=c.receive(reply_text('actuator',0),.13,10100,boot_us=10_100_000)
    assert snap and snap['timestamp_us']==10_000_000


def test_actor_microsecond_age_boundary_matches_input_reducer():
    c=p.MissionAuditCollector(ground_status_max_age=1.);c.action(0,True)
    assert c.receive(reply_text('actuator',0),1.01,11000,boot_us=11_000_001) is None
    assert c.last_snapshot is None
    assert c.noncurrent_actuator_reads[-1]['reason']=='outside_ground_status_age'


def test_far_future_or_unsafe_future_actor_reply_still_fails_closed():
    c=p.MissionAuditCollector(ground_status_max_age=1.);c.action(0,True)
    assert c.receive(reply_text('actuator',0),.01,9900,boot_us=9_900_000) is None
    assert c.failures and c.last_snapshot is None
    c=p.MissionAuditCollector(ground_status_max_age=1.);c.action(0,True)
    with pytest.raises(RuntimeError,match='Unsafe'):
        c.receive(reply_text('actuator',0).replace('armed: False','armed: True'),.01,9975,boot_us=9_975_000)
    assert c.disabled


@pytest.mark.parametrize('limit',[0.,2.,True,float('nan'),float('inf'),'1'])
def test_ground_tolerance_has_only_bounded_reviewed_profiles(limit):
    with pytest.raises(ValueError,match='0.5 or 1.0'):
        p.MissionAuditCollector(ground_status_max_age=limit)


def test_native_repeated_publication_preserves_source_time_and_expires_when_frozen():
    from rc_watch import NativeRCMonitor
    c=p.MissionAuditCollector(native_rc=NativeRCMonitor())
    text=reply_text('actuator',.01)
    c.action(.01,True,full_audit=False)
    first=c.receive(text,.02,10020)
    assert first
    # Same current read, same source value; never rewrite source time to now.
    c.key='actuator';c.transaction.begin(p.COMMANDS['actuator'],.6)
    repeated=c.receive(text,.61,10610)
    assert repeated['timestamp_us']==first['timestamp_us']
    assert repeated['at']==.61 and c.live_lockdown(.62) is True
    assert c.live_lockdown(1.011) is None  # Source frozen beyond 1s despite a recent reply.
    c.key='actuator';c.transaction.begin(p.COMMANDS['actuator'],1.52)
    assert c.receive(text,1.53,11530) is None
    assert c.last_snapshot['at']==.61


def test_native_conflicting_same_timestamp_cannot_refresh_output_state():
    from rc_watch import NativeRCMonitor
    c=p.MissionAuditCollector(native_rc=NativeRCMonitor());c.action(0,True)
    assert deliver(c,'actuator',.01)
    c.key='actuator';c.transaction.begin(p.COMMANDS['actuator'],.1)
    text=reply_text('actuator',.01).replace('manual_lockdown: False','manual_lockdown: True')
    assert c.receive(text,.11,10110) is None
    assert 'Conflicting actuator values' in c.failures[-1]['reason']
    assert c.last_snapshot['at']==.01


def test_recorded_periodic_manual_sample_does_not_force_redundant_refresh():
    from rc_watch import NativeRCMonitor
    from test_pocket_wifi_delta import topic_text
    # 2026-11-09 ground attempt: manual reply 7.668055175, source 222686969;
    # at failure 7.985695594 TIMESYNC upper bound was 223192098.912 us.
    rc=NativeRCMonitor();now=7.835695594
    rc.observe_clock(223042098.912,now)
    for topic in rc.TOPICS:rc.observe_topic(topic,topic_text(topic,now=222950000),now-.03)
    rc.observe_topic('manual_control_setpoint',topic_text('manual_control_setpoint',now=222686969),7.668055175)
    c=p.MissionAuditCollector(native_rc=rc)
    c.last_snapshot=dict(at=now,fc_age_at_receive_s=.05,lockdown=False)
    c.queue.clear();c.queue.extend(('selector','native_input_rc'))
    assert c.action(now,True)=='selector'
    # Keep the pending shell reply exclusive even when another topic is due.
    assert c.action(now+.03,True) in (None,'poll')


def test_manual_timing_separates_500ms_reply_from_200ms_publication():
    from rc_watch import NativeRCMonitor
    from test_pocket_wifi_delta import topic_text
    rc=NativeRCMonitor();now=7.985695594
    rc.observe_clock(223192098.912,now)
    for topic in rc.TOPICS:rc.observe_topic(topic,topic_text(topic,now=223100000),now-.02)
    rc.observe_topic('manual_control_setpoint',topic_text('manual_control_setpoint',now=222686969),7.668055175)
    status=rc.status(now,max_age=.5);timing=status['topic_timing']['manual_control_setpoint']
    assert timing['host_age_s']==pytest.approx(.317640419)
    assert timing['source_age_upper_s']==pytest.approx(.505129912)
    assert timing['reply_limit_s']==.5 and timing['source_limit_s']==.7
    assert not timing['problems']
    assert status['native_input_observed']
    rc.observe_topic('manual_control_setpoint',topic_text('manual_control_setpoint',now=223150000),now)
    assert rc.status(now,max_age=.5)['native_input_observed']
    rc.observe_clock(223900001,now+.3)
    # A fresh reply containing frozen data remains invalid.
    rc.observe_topic('manual_control_setpoint',topic_text('manual_control_setpoint',now=223150000),now+.3)
    assert 'source_stale_or_future' in rc.topic_timing('manual_control_setpoint',now+.3)['problems']


def test_recorded_long_audit_uses_relaxed_failsafe_reply_budget():
    import json
    from copy import deepcopy
    from rc_watch import NativeRCMonitor
    evidence=json.loads((Path(__file__).parent/'fixtures/native_shell_timing_20261109.json').read_text())
    failed=evidence['failure_native'];failure_at=evidence['failure_context']['now']
    now=3.947  # After the latest manual reply, before the long flags read.
    rc=NativeRCMonitor();rc.topics=deepcopy(failed['raw_snapshots'])
    upper=(failed['input_rc']['timestamp']+failed['topic_timing']['input_rc']['source_age_upper_s']*1e6
           -(failure_at-now)*1e6)
    rc.observe_clock(upper,now)
    c=p.MissionAuditCollector(native_rc=rc)
    c.last_snapshot=dict(at=rc.topics['actuator_armed']['received_at'],lockdown=False,
        fc_age_at_receive_s=.32153978)
    c.queue.clear();c.queue.append('flags_0')
    slack=rc.topic_timing('failsafe_flags',now)['query_headroom_s']
    assert .66<slack<.68  # The new 1s reply budget, not the old 500ms limit.
    assert c.query_budget('flags_0')+c.query_budget('native_failsafe_flags')<slack
    assert c.action(now,True)!='native_failsafe_flags'
    assert c.action(now+.04,True) in (None,'poll')  # Still one in-flight query.


def test_native_idle_does_not_spin_redundant_actuator_queries():
    from rc_watch import NativeRCMonitor
    from test_pocket_wifi_delta import topic_text
    rc=NativeRCMonitor();rc.observe_clock(11000000,1.)
    for topic in rc.TOPICS:rc.observe_topic(topic,topic_text(topic,now=11000000),1.)
    c=p.MissionAuditCollector(native_rc=rc);c._full_audit_enabled=False
    c.queue.clear();c.last_snapshot=dict(at=1.,fc_age_at_receive_s=0.,lockdown=False)
    assert c.action(1.,True,full_audit=False) is None


@pytest.mark.parametrize('requested,finished,reuse',[(1.1,False,True),(.9,False,False),(1.1,True,False)])
def test_fast_status_reuse_stays_inside_open_audit_window(requested,finished,reuse):
    from rc_watch import NativeRCMonitor
    c=p.MissionAuditCollector(native_rc=NativeRCMonitor())
    c.row={'selector':{},'_finished':finished};c.row_started=1.
    c.queue.extend(('local_position','commander'))
    c.key='critical_status';c.requested_at=requested
    c._accept(p.parse('critical_status',reply_text('critical_status',1.2)),1.2,11200)
    assert ('local' in c.row) is reuse
    assert ('commander' in c.row) is reuse
    assert ('local_position' not in c.queue) is reuse
    assert ('commander' not in c.queue) is reuse


@pytest.mark.parametrize('complete',[False,True])
def test_only_complete_failsafe_print_can_replace_audit_query(complete):
    from rc_watch import NativeRCMonitor
    from test_pocket_wifi_delta import topic_text
    c=p.MissionAuditCollector(native_rc=NativeRCMonitor())
    c.row={'selector':{}};c.row_started=1.;c.queue.append('failsafe')
    c.key='native_failsafe_flags';c.requested_at=1.1
    c.transaction.begin(p.COMMANDS[c.key],1.1)
    if complete:
        text=reply_text('failsafe',1.2)
    else:
        text='nsh> '+p.COMMANDS[c.key].decode()+topic_text('failsafe_flags',now=11200000)+'\nnsh> '
    c.receive(text,1.2,11200)
    assert ('failsafe' in c.row) is complete
    assert ('failsafe' not in c.queue) is complete


def test_slow_diagnostic_does_not_abort_at_two_seconds():
    from rc_watch import NativeRCMonitor
    c=p.MissionAuditCollector(native_rc=NativeRCMonitor())
    c.row={'selector':{}};c.row_started=0.
    assert c.action(2.039,True)=='actuator'
    deliver(c,'actuator',2.05)
    assert not c.failures
    assert not c.disabled
    assert c.action(2.06,True,restoration_only=True)=='actuator'


@pytest.mark.parametrize('key',['local_position','commander'])
def test_between_audit_fast_reply_does_not_open_an_unbracketed_row(key):
    from rc_watch import NativeRCMonitor
    c=p.MissionAuditCollector(native_rc=NativeRCMonitor())
    c.key=key;c.requested_at=1.
    c._accept(p.parse(key,reply_text(key,1.1)),1.1,11100)
    assert c.critical_samples and c.status_updates
    assert not c.row and c.row_started is None
