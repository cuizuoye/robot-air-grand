"""Run the real monitor loop against fake ROS/input and MAVLink wire bytes.

No real device opens, subprocesses, network or ROS context are used here.
"""
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import trs_runtime_monitor as p
from estop_only_policy import LISTENER
from test_mission_audit_collector import reply_text


@pytest.fixture
def reactor(tmp_path, monkeypatch):
    def run(scenario='healthy', home_at=4.0, seconds=30, until_ready=False, baud=57600,
            tolerate_isolated_frames=False, mission_input_shadow=False, mission_audit_shadow=False,
            mission_telemetry_rates=False, ground_status_max_age=.5, physical_mode='usb-only'):
        clock = NS(value=0.0)
        sent = []
        fc = m.MAVLink(None, srcSystem=1, srcComponent=1)
        outbound = m.MAVLink(None)

        class FakeSerial:
            def __init__(self):
                self.queue = bytearray()
                self.queries = 0
                self.locked = False
                self.closed = False
                self.bad_local_sent = False
                self.intervals={32:1000000,230:2000000,245:1000000}
                self.next_stream={key:0. for key in self.intervals}

            def __enter__(self):
                return self

            def __exit__(self, *exc):
                self.closed = True

            def reset_input_buffer(self):
                self.queue.clear()

            def packet(self, msg):
                wire = msg.pack(fc)
                fc.seq = (fc.seq+1) % 256
                return wire

            def append(self, msg):
                self.queue.extend(self.packet(msg))

            def read(self, size):
                clock.value += .1
                if scenario!='heartbeat_gap' or not 10<=clock.value<=12.5:
                    self.append(fc.heartbeat_encode(m.MAV_TYPE_QUADROTOR, m.MAV_AUTOPILOT_PX4, 0, 0, 0))
                self.append(fc.attitude_quaternion_encode(int(clock.value*1000), 1, 0, 0, 0, 0, 0, 0))
                if mission_telemetry_rates:
                    for key in self.intervals:
                        if clock.value>=self.next_stream[key]:
                            boot=int(clock.value*1000)
                            message=(fc.local_position_ned_encode(boot,0,0,0,0,0,0) if key==32
                                else fc.estimator_status_encode(boot*1000,0,0,0,0,0,0,0,0,0) if key==230
                                else fc.extended_sys_state_encode(0,m.MAV_LANDED_STATE_ON_GROUND))
                            self.append(message)
                            self.next_stream[key]=clock.value+self.intervals[key]/1e6-.001
                if scenario=='bad_local_with_home' and clock.value>=home_at+.09 and not self.bad_local_sent:
                    self.append(fc.local_position_ned_encode(int(clock.value*1000),float('nan'),0,0,0,0,0))
                    self.bad_local_sent=True
                # Keep wire order: attitude precedes the pending shell response
                # in the next read, as it does in a normal streaming transport.
                data = bytes(self.queue[:size])
                del self.queue[:size]
                return data

            def write(self, wire):
                for msg in outbound.parse_buffer(wire) or []:
                    sent.append((clock.value, msg))
                    kind = msg.get_type()
                    if kind == 'COMMAND_LONG' and msg.command == 512:
                        self.append(fc.autopilot_version_encode(0, (1<<24)+(14<<16)+(3<<8),
                            0, 0, 54, bytes(8), bytes(8), bytes(8), 0, 0, 1, bytes.fromhex(p.UID2)))
                    elif kind == 'COMMAND_LONG' and msg.command == 185:
                        result = m.MAV_RESULT_DENIED if scenario == 'reject_ack' else m.MAV_RESULT_ACCEPTED
                        if result == m.MAV_RESULT_ACCEPTED:
                            self.locked = True
                        if scenario != 'drop_ack':
                            self.append(fc.command_ack_encode(185, result, target_system=250, target_component=202))
                    elif kind=='COMMAND_LONG' and msg.command==510:
                        key=int(msg.param1)
                        self.append(fc.message_interval_encode(interval_us=self.intervals[key],message_id=key))
                    elif kind=='COMMAND_LONG' and msg.command==511:
                        key=int(msg.param1);self.intervals[key]=int(msg.param2) if msg.param2>0 else 0
                        self.append(fc.command_ack_encode(511,0,target_system=250,target_component=202))
                    elif kind == 'PARAM_REQUEST_READ':
                        name = msg.param_id.rstrip('\0')
                        value = p.PARAMS[name]
                        # PX4 integer PARAM values are bytewise encoded.
                        encoded = struct.unpack('<f', struct.pack('<i', int(value)))[0]
                        self.append(fc.param_value_encode(name.encode(), encoded, m.MAV_PARAM_TYPE_INT32, 9, 0))
                    elif kind == 'SERIAL_CONTROL' and msg.count:
                        payload=bytes(msg.data[:msg.count])
                        if mission_audit_shadow and payload!=LISTENER:
                            key=next(key for key,value in p.audit_policy.COMMANDS.items() if value==payload)
                            # This fake FC uses boot seconds == fake owner time,
                            # whereas the parser fixture uses owner time +10.
                            text=reply_text(key,clock.value-10).encode()
                            for index in range(0,len(text),70):
                                chunk=text[index:index+70]
                                self.append(fc.serial_control_encode(10,0,0,0,len(chunk),chunk.ljust(70,b'\0')))
                            continue
                        assert payload == LISTENER
                        self.queries += 1
                        if scenario == 'missing_after_baseline' and self.queries > 1:
                            continue
                        stamp = max(1,int((clock.value-(.7 if scenario=='cached_actuator' else 0))*1e6))
                        text = ('nsh> listener actuator_armed -n 1\nTOPIC: actuator_armed\n'
                                f' timestamp: {stamp}\n armed: False\n lockdown: {self.locked}\n'
                                ' manual_lockdown: False\n force_failsafe: False\n\nnsh> ').encode()
                        for index in range(0, len(text), 70):
                            chunk = text[index:index+70]
                            packet = bytearray(self.packet(fc.serial_control_encode(
                                10, 0, 0, 0, len(chunk), chunk.ljust(70, b'\0'))))
                            if scenario == 'corrupt_second' and self.queries == 2 and index == 70:
                                packet[-1] ^= 1  # Deliberately corrupt a real MAVLink frame CRC.
                            self.queue.extend(packet)
                return len(wire)

        port = FakeSerial()
        pending_events = [(home_at, 1), (home_at+.2, 0)]

        def input_read(fd, size):
            if pending_events and clock.value >= pending_events[0][0]:
                _, value = pending_events.pop(0)
                return p.EVENT.pack(0, 0, 1, p.HOME, value)
            raise BlockingIOError

        class FakeNode:
            def __init__(self, name):
                pass

            def create_subscription(self, *a, **kw):
                return object()

            def get_clock(self):
                return NS(now=lambda: NS(nanoseconds=int(clock.value*1e9)))

            def destroy_node(self):
                pass

        class Gate:
            def __init__(self, **kw):
                self.accepted, self.rejected = 1, 0
                self.last = NS(position=[0, 0, 0], quaternion=[0, 0, 0, 1],
                               stamp_ns=1,received_monotonic_ns=0)

            def fresh(self, *a):
                if scenario=='missing_vicon':return False
                if scenario=='late_vicon':return clock.value>=12
                return True

        def transform(*args):
            if scenario=='bad_tilt':return [0,0,0],[0,1,0,0]
            if scenario=='source_flip' and clock.value>=3:return [0,0,0],[1,0,0,0]
            return [0,0,0],[0,0,0,1]

        ros = NS(init=lambda **kw: None, ok=lambda: True,
                 shutdown=lambda: None, spin_once=lambda *a, **kw: None)
        for name, module in {
            'rclpy': ros, 'rclpy.node': NS(Node=FakeNode),
            'rclpy.qos': NS(QoSProfile=lambda **kw: None,
                ReliabilityPolicy=NS(BEST_EFFORT=1), DurabilityPolicy=NS(VOLATILE=1)),
            'geometry_msgs.msg': NS(PoseStamped=object),
            'uav_coord_bridge.mavlink_vision': NS(PoseGate=Gate,
                transform_rigid_pose=transform),
        }.items():
            monkeypatch.setitem(sys.modules, name, module)
        monkeypatch.setattr(p, 'time', NS(monotonic=lambda: clock.value,
                                        monotonic_ns=lambda: int(clock.value*1e9)))
        def serial_open(path, speed, **kwargs):
            assert path == '/fake/TRS' and speed == baud
            return port
        monkeypatch.setattr(p, 'serial', NS(Serial=serial_open))
        monkeypatch.setattr(p, 'os', NS(O_RDONLY=0, O_NONBLOCK=0,
            open=lambda *a: 999, close=lambda *a: None, read=input_read,
            fsync=lambda *a:None,
            stat=lambda *a: NS(st_rdev=1,st_ino=1), fstat=lambda *a: NS(st_rdev=1,st_ino=1)))
        def pad_identity(path):
            replaced=scenario=='pad_replaced' and clock.value>=3
            return {'rdev':1,'inode':2 if replaced else 1,
                    'sysfs_device':'new' if replaced else 'initial'}
        monkeypatch.setattr(p,'gamepad_identity',pad_identity)
        monkeypatch.setattr(p, 'fcntl', NS(ioctl=lambda *a: bytes(96)))
        monkeypatch.setattr(p, 'subprocess', NS(run=lambda *a, **kw: NS(returncode=1)))
        monkeypatch.setattr(p, 'find_pad', lambda: {'path': '/fake/home-input'})
        monkeypatch.setattr(p, 'check_physical', lambda mode: None)
        def evidence(*args):
            if scenario=='calibration_changed' and clock.value>=3:
                raise ValueError('candidate_hash_changed')
            return {'active_revision_id':'offline-test-only'}
        monkeypatch.setattr(p, 'inspect_evidence', evidence)
        class Console:
            def publish(self,text):pass
            def close(self):pass
        monkeypatch.setattr(p,'LatestStatusPrinter',Console)
        heading, board = tmp_path/'heading.json', tmp_path/'board.json'
        heading.write_text(json.dumps({'body_orientation_in_rigid_xyzw_candidate': [0, 0, 0, 1]}))
        board.write_text(json.dumps({'board_center_in_rigid_m_candidate': [0, 0, 0]}))
        args = NS(physical_mode=physical_mode, props_installed_monitor_stop_only_confirmed=True,
                  trs_port='/fake/TRS', baud=baud, heading=heading, board=board, static_proof=tmp_path/'unused',
                  seconds=seconds, output=tmp_path/'report.json', status_log=tmp_path/'status.jsonl',
                  latch_file=tmp_path/'latch.json',until_ready=until_ready)
        args.tolerate_isolated_frames=tolerate_isolated_frames
        args.mission_input_shadow=mission_input_shadow
        args.mission_audit_shadow=mission_audit_shadow
        args.mission_telemetry_rates=mission_telemetry_rates
        args.ground_status_max_age=ground_status_max_age
        args.rates_recovery_journal=tmp_path/'rates_recovery.json' if mission_telemetry_rates else None
        code = p.run(args)
        report = json.loads(args.output.read_text())
        assert report['trs_baud'] == baud
        statuses = [json.loads(line) for line in args.status_log.read_text().splitlines()]
        assert port.closed
        fatal=scenario in ('bad_local_with_home','pad_replaced')
        if scenario=='bad_local_with_home':assert report['error']=='Non-finite local telemetry'
        elif scenario=='pad_replaced':assert report['error']=='Gamepad device changed or re-enumerated; new session required'
        else:
            if not mission_telemetry_rates and not (until_ready and report['readiness_completed']):
                assert report['duration_s'] >= seconds
            assert report['error'] is None
        # Every outbound wire message still obeys the stop-only whitelist.
        for at, msg in sent:
            if mission_telemetry_rates and msg.get_type()=='COMMAND_LONG' and msg.command in (510,511):
                from trs_message_interval_trial import allowed as interval_allowed
                original={32:1000000,230:2000000,245:1000000}[int(msg.param1)]
                assert interval_allowed(msg,safe=True,locked=True,original=original)
                continue
            assert p.allowed(msg, identity_fresh=True, home_latched=at >= home_at,
                             audit_shadow=mission_audit_shadow)
            if msg.get_type() == 'COMMAND_LONG' and msg.command == 185:
                assert at >= home_at and msg.param1 == 2
        assert report['pose_messages_sent'] == report['motion_targets_sent'] == report['parameter_writes_sent'] == 0
        assert report['flight_adapter_enabled'] is False
        assert report['monitor_running_after_exit'] is False
        assert report['last_status']['home_input_monitor_running'] is False
        if not fatal:assert report['home_events'][-1]['value'] == 0
        assert json.loads(args.latch_file.read_text())['latched'] is True
        return code, report, statuses, sent
    return run


def test_mission_shadow_observes_existing_owner_without_new_transmit_permissions(reactor):
    _,report,statuses,sent=reactor(mission_input_shadow=True)
    shadow=report['mission_input_shadow']
    assert shadow['core_phase']=='PREPARE'
    assert not shadow['observation_complete'] and not shadow['flight_authorized']
    assert shadow['audit_rows']==shadow['mission_messages_sent']==0
    assert 'current_xyz_yaw_fusion_not_verified' in shadow['missing_conditions']
    assert not shadow['full_audit_producer_attached']
    assert all(s['mission_input_shadow']['mission_messages_sent']==0 for s in statuses)
    assert not any(msg.get_type() in ('VISION_POSITION_ESTIMATE','SET_POSITION_TARGET_LOCAL_NED','PARAM_SET')
                   or msg.get_type()=='COMMAND_LONG' and msg.command in (176,400,21,22)
                   for _,msg in sent)


def test_shadow_cannot_promote_crc_recovery_to_fusion_or_flight_readiness(reactor):
    _,report,_,_=reactor('corrupt_second',home_at=8,tolerate_isolated_frames=True,
                       mission_input_shadow=True)
    assert report['mission_input_shadow']['crc_discarded_frames']>0
    assert not report['mission_input_shadow']['observation_complete']
    assert report['mission_input_shadow']['mission_messages_sent']==0


def test_real_monitor_loop_uses_one_nonblocking_audit_owner_and_no_extra_control_scope(reactor):
    _,report,statuses,sent=reactor(mission_audit_shadow=True)
    shadow=report['mission_input_shadow']
    assert shadow['full_audit_producer_attached'] and shadow['fixed_audit_reads']>20
    assert shadow['mission_messages_sent']==0 and not shadow['flight_authorized']
    payloads=[bytes(msg.data[:msg.count]) for _,msg in sent if msg.get_type()=='SERIAL_CONTROL' and msg.count]
    assert p.audit_policy.COMMANDS['commander'] in payloads
    assert p.audit_policy.COMMANDS['local_position'] in payloads
    assert report['home_release_lock_verified']  # HOME still serviced during the audit pipeline.
    assert all(s['mission_input_shadow']['mission_messages_sent']==0 for s in statuses)


def test_temporary_rates_are_restored_before_same_serial_owner_closes(reactor):
    code,report,statuses,sent=reactor(mission_audit_shadow=True,mission_telemetry_rates=True)
    rates=report['temporary_telemetry_rates']
    assert code==0 and rates['restoration_verified'] and rates['measured_profile_verified']
    assert rates['original_intervals_us']=={'32':1000000,'230':2000000,'245':1000000}
    sets=[msg for _,msg in sent if msg.get_type()=='COMMAND_LONG' and msg.command==511]
    assert len(sets)==6 and all(msg.param1!=0 for msg in sets)
    assert [int(msg.param2) for msg in sets[-3:]]==[1000000,2000000,1000000]
    assert report['mission_input_shadow']['full_audit_producer_attached']
    assert report['pose_messages_sent']==report['parameter_writes_sent']==0


def test_rate_restoration_still_attempted_after_audit_input_fault(reactor):
    _,report,_,sent=reactor('heartbeat_gap',mission_audit_shadow=True,mission_telemetry_rates=True)
    assert report['temporary_telemetry_rates']['restoration_verified']
    assert not report['continuous_monitor_inputs_healthy']  # Timely data != healthy heartbeat.
    assert report['first_fault_context']['reason']=='fc_heartbeat_stale_after_ready'


@pytest.mark.parametrize('baud', [57600, 115200])
def test_explicit_baud_reaches_serial_and_status_reports(reactor, baud):
    _, report, statuses, _ = reactor(baud=baud)
    assert report['trs_baud'] == baud
    assert all(status['trs_baud'] == baud for status in statuses)


def test_reused_event_number_aborts_before_home_from_replacement(reactor):
    code,report,_,sent=reactor('pad_replaced',home_at=4)
    assert code==2 and report['duration_s']<4
    assert not report['home_events'] and not report['home_release_lock_verified']
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in sent)
    assert report['gamepad_generation_identity']['inode']==1
    assert not report['monitor_running_after_exit']


def test_heartbeat_gap_retains_diagnostic_context_and_never_clears_after_recovery(reactor):
    code,report,_,_=reactor('heartbeat_gap')
    assert code==2 and report['home_release_lock_verified']
    context=report['first_fault_context']
    assert context['reason']=='fc_heartbeat_stale_after_ready'
    assert context['heartbeat_age_s']>=2
    assert context['received_bad_data_count']==0  # No invented CRC/RF root cause.
    assert context['recent_heartbeats']
    assert report['last_status']['identity_fresh_disarmed']
    assert report['last_status']['monitor_fault_latched']==context['reason']
    assert report['last_status']['heartbeat_max_arrival_gap_s']>2
    assert report['last_status']['heartbeat_age_s']<2
    assert not report['continuous_monitor_inputs_healthy']


def test_healthy_home_lock_and_monitor_continues_to_deadline(reactor):
    code, report, statuses, _ = reactor()
    assert code == 0
    assert report['home_release_lock_verified']
    assert report['continuous_monitor_inputs_healthy']
    assert any(s['home_release_lock_verified'] and s['home_input_monitor_running'] for s in statuses)


def test_corrupt_frame_does_not_kill_home_or_erase_fault_after_recovery(reactor):
    code, report, statuses, _ = reactor('corrupt_second')
    assert code == 2  # Remains degraded even though a later query succeeded.
    assert report['home_release_lock_verified']
    assert report['actuator_query_faults'][0]['reason'] == 'actuator_reply_damaged_packet'
    assert report['last_status']['monitor_fault_latched'] == 'actuator_reply_damaged_packet'
    assert not report['continuous_monitor_inputs_healthy']
    assert report['bad_packet_examples']
    assert any(s['home_input_monitor_running'] and s['monitor_fault_latched'] for s in statuses)


def test_opt_in_damage_recovery_requires_new_proof_without_latching_isolated_crc(reactor):
    code,report,statuses,_=reactor('corrupt_second',home_at=8,
                                  tolerate_isolated_frames=True)
    assert code==0 and report['home_release_lock_verified']
    assert report['tolerate_isolated_frames']
    assert report['actuator_query_faults'][0]['reason']=='actuator_reply_damaged_packet'
    assert report['last_status']['monitor_fault_latched'] is None
    assert not report['actuator_queries_disabled']
    failed_at=report['actuator_query_faults'][0]['at']
    assert any(row['requested_at']>failed_at for row in report['snapshots'])
    assert report['bad_packet_examples']  # Damage history is never erased.
    assert not report['flight_adapter_enabled']


@pytest.mark.parametrize('scenario',['heartbeat_gap','missing_after_baseline','drop_ack'])
def test_opt_in_does_not_tolerate_stale_link_missing_state_or_unconfirmed_home(reactor,scenario):
    code,report,_,_=reactor(scenario,tolerate_isolated_frames=True)
    assert code==2 and report['last_status']['monitor_fault_latched']
    assert not report['continuous_monitor_inputs_healthy']


def test_opt_in_until_ready_still_requires_fresh_recovered_state_and_home(reactor):
    code,report,_,_=reactor('corrupt_second',home_at=8,until_ready=True,
                           tolerate_isolated_frames=True)
    assert code==0 and report['readiness_completed']
    assert report['completion_snapshot']['live_lockdown'] is True
    assert report['home_release_lock_verified']
    assert report['actuator_query_faults']
    assert not report['last_status']['session']['flight_authorized']


def test_missing_reply_disables_diagnostics_but_later_home_still_sent(reactor):
    code, report, statuses, sent = reactor('missing_after_baseline', home_at=16)
    assert code == 2
    assert report['actuator_queries_disabled']
    assert report['home_acks'][0]['result'] == m.MAV_RESULT_ACCEPTED
    assert not report['home_release_lock_verified']  # ACK is not state proof.
    assert report['last_status']['live_lockdown'] is None
    assert not report['last_status']['home_lock_currently_confirmed']
    assert any(s['elapsed_s'] >= 25 and s['home_input_monitor_running'] for s in statuses)
    locks = [msg for _, msg in sent if msg.get_type() == 'COMMAND_LONG' and msg.command == 185]
    assert len(locks) == 1


def test_negative_lock_ack_cannot_pass_and_does_not_close_input_listener(reactor):
    code, report, statuses, _ = reactor('reject_ack')
    assert code == 2
    assert not report['home_release_lock_verified']
    assert report['last_status']['monitor_fault_latched'] == 'home_output_lock_rejected'
    assert any(s['elapsed_s'] >= 25 and s['home_input_monitor_running'] for s in statuses)


def test_missing_ack_keeps_listener_and_retry_limit_without_false_success(reactor):
    code, report, statuses, sent = reactor('drop_ack')
    assert code == 2
    assert not report['home_release_lock_verified']
    assert report['last_status']['monitor_fault_latched'] == 'home_lock_confirmation_timeout'
    assert not report['last_status']['home_lock_currently_confirmed']
    assert not report['home_acks']
    locks = [msg for _, msg in sent if msg.get_type() == 'COMMAND_LONG' and msg.command == 185]
    assert [msg.confirmation for msg in locks] == [0, 1, 2]
    assert any(s['elapsed_s'] >= 25 and s['home_input_monitor_running'] for s in statuses)


def test_early_home_is_latched_until_identity_without_losing_the_listener(reactor):
    code,report,statuses,sent=reactor(home_at=.1)
    assert code==0 and report['home_release_lock_verified']
    assert report['duration_s']>=30
    assert any(msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in sent)
    assert any(s['home_input_monitor_running'] and s['session']['phase']=='STATIC_BENCH_INPUTS_OBSERVED'
               for s in statuses)


def test_home_is_sent_before_unrelated_telemetry_parser_failure(reactor):
    code,report,_,sent=reactor('bad_local_with_home',home_at=4.)
    assert code==2
    assert report['home_events'][0]['value']==1
    assert any(msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in sent)
    assert report['last_status']['session']['phase']=='CLOSED'
    assert not report['last_status']['session']['flight_authorized']


def test_session_status_names_the_missing_home_then_closes_without_authority(reactor):
    code,report,statuses,_=reactor(home_at=12.)
    assert code==0
    assert any(s['session']['phase']=='WAIT_HOME_CHECK' for s in statuses)
    assert any(s['session']['phase']=='STATIC_BENCH_INPUTS_OBSERVED' for s in statuses)
    assert report['last_status']['session']['phase']=='CLOSED'
    assert all(not s['session']['flight_authorized'] for s in statuses)


def test_parameter_refresh_is_spread_instead_of_enqueued_as_a_burst(reactor):
    _,_,_,sent=reactor()
    times=[at for at,msg in sent if msg.get_type()=='PARAM_REQUEST_READ']
    assert len(times)>=len(p.PARAMS)
    assert all(b-a>=.099 for a,b in zip(times,times[1:]))


def test_ground_one_second_profile_does_not_feed_older_samples_as_mission_state(reactor):
    _,report,statuses,sent=reactor('cached_actuator',mission_audit_shadow=True,
                                  ground_status_max_age=1.)
    assert report['last_status']['ground_status_max_age_s']==1.
    assert any(row['live_lockdown'] is True and row['strict_500ms_lockdown'] is None for row in statuses)
    assert any(row['fc_age_at_receive_s']>.5 for row in report['snapshots'])
    reasons=report['mission_input_shadow']['missing_conditions']
    assert 'actuator_state_missing_or_stale' in reasons
    assert not any('input_shadow_invalid' in reason for reason in reasons)
    assert not report['mission_input_shadow']['observation_complete']
    assert not report['flight_adapter_enabled'] and not report['motion_targets_sent']
    assert all(msg.command in (185,512) for _,msg in sent if msg.get_type()=='COMMAND_LONG')


@pytest.mark.parametrize('audit,mode',[(False,'usb-only'),(True,'battery-only')])
def test_relaxed_ground_profile_is_rejected_outside_usb_audit_scope(reactor,audit,mode):
    with pytest.raises(ValueError,match='USB-only fixed-audit'):
        reactor(mission_audit_shadow=audit,physical_mode=mode,ground_status_max_age=1.)


def test_until_ready_finishes_early_only_after_current_conditions_are_stable(reactor):
    code,report,_,_=reactor(until_ready=True)
    assert code==0 and report['readiness_completed']
    assert report['duration_s']<30
    assert report['completion_snapshot']['session']['static_bench_inputs_observed']
    assert not report['monitor_running_after_exit']
    assert report['last_status']['session']['phase']=='CLOSED'


def test_until_ready_waits_for_vicon_connected_later(reactor):
    code,report,_,_=reactor('late_vicon',until_ready=True)
    assert code==0 and report['readiness_completed']
    assert 13<=report['duration_s']<30


def test_until_ready_timeout_with_missing_condition_is_not_success(reactor):
    code,report,_,_=reactor('missing_vicon',until_ready=True)
    assert code==2 and not report['readiness_completed']
    assert report['duration_s']>=30 and report['completion_snapshot'] is None


def test_until_ready_does_not_clear_fault_and_exit_success_after_recovery(reactor):
    code,report,_,_=reactor('corrupt_second',until_ready=True)
    assert code==2 and not report['readiness_completed']
    assert report['last_status']['monitor_fault_latched']


@pytest.mark.parametrize('scenario',['bad_tilt','source_flip','calibration_changed'])
def test_bad_mapping_or_attitude_blocks_ready_but_home_monitor_continues(reactor,scenario):
    code,report,statuses,sent=reactor(scenario,until_ready=True)
    assert code==2 and not report['readiness_completed']
    assert report['home_release_lock_verified']
    assert any(s['home_input_monitor_running'] and s['session']['phase']=='DEGRADED' for s in statuses)
    assert any(msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in sent)
    if scenario=='calibration_changed':
        assert not report['calibration_current'] and report['calibration_errors']
    else:assert report['attitude_consistency']['fault']
