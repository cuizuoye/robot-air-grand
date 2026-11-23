"""Resource-injected maintenance lifecycle. Not itself a live resource factory.

No CLI, port open, ROS/SDK/node, thread, HOME sender or parameter transaction.
The existing owner supplies ORIGINAL observations and services HOME/CRC/RX
while its pump runs. Its bind_recording hook must use _write_owner_packet;
it must not fall back to the static/dynamic-parameter runner on detach/fault.
Logger I/O uses its own, distinct, already-owned USB channel. Pumps must
service both owners without a second reader for either channel.

Physical adapters and one-use permission live in ev_delay_joint_capture.py.
The legacy run_live entry remains closed; constructing this host opens nothing.
"""
from copy import deepcopy

from ekf_ground_acceptance import result
from ev_delay_maintenance import MaintenanceObservation, MaintenanceRecordingSession, WINDOW_NS
import static_ulog_capture_actions as logs
import static_ulog_capture_policy as log_policy

LIVE_ENTRY_ENABLED = False
HOME_WAIT_NS = 90_000_000_000
SELECTION_QUERY = b'listener sensor_selection -n 1\n'
VISUAL_QUERY = b'listener vehicle_visual_odometry -n 1\n'


def maintenance_usb_allowed(message, **context):
    """Original Logger gate + fixed selection and visual receipt queries.

    Not installed in the production USB tool. No arbitrary shell extension,
    parameter write, HOME/mode command or file writer is added.
    """
    if log_policy.allowed(message, **context):
        return True
    if (context.get('safe') is True and context.get('phase') == 'RECORDING'
            and message.get_type() == 'SERIAL_CONTROL'
            and (message.device,message.flags,message.timeout,message.baudrate)==(10,2,0,0)
            and message.count == len(VISUAL_QUERY) and bytes(message.data)==VISUAL_QUERY.ljust(70,b'\0')):
        return True
    return bool(context.get('safe') is True and context.get('phase') in
        ('PRECHECK', 'STARTING', 'STOPPING', 'VERIFYING')
        and message.get_type() == 'SERIAL_CONTROL'
        and (message.device, message.flags, message.timeout, message.baudrate) == (10, 2, 0, 0)
        and message.count == len(SELECTION_QUERY)
        and bytes(message.data) == SELECTION_QUERY.ljust(70, b'\0'))


class ObservationCache:
    """Owner publishes explicit original evidence, not a freshness generator.

    Snapshot changes ONLY the assessment time. In particular this does not
    retime a UID/version reply, lock/clock, attitude, Vicon or HOME proof.
    Production conversion from owner's verified state remains separately
    reviewable; callers cannot use this class to bypass the existing gate.
    """
    def __init__(self):
        self.latest = None

    def publish(self, observation):
        if type(observation) is not MaintenanceObservation:
            raise TypeError('Original MaintenanceObservation required')
        if self.latest is not None and observation.now_ns < self.latest.now_ns:
            raise ValueError('Owner observation clock regressed')
        self.latest = deepcopy(observation)

    def snapshot(self, now_ns):
        from dataclasses import replace
        if self.latest is None:
            raise RuntimeError('Current owner observation missing; no synthetic defaults')
        if type(now_ns) is not int or now_ns < self.latest.now_ns:
            raise ValueError('Assessment precedes original observation')
        return replace(self.latest, now_ns=now_ns)


def run_live(*args, **kwargs):
    raise PermissionError('Maintenance live resource entry remains closed; offline injection only')


class MaintenanceHost:
    """One bounded injected run; callbacks are not hardware authorization.

    owner contract (no resources opened here):
      port, observation(now_ns), bind_recording(session, stop_at_ns), pump(),
      stop_pose_writes(), stop_inputs(), detach_recording(), close_owned(),
      verify_configuration() (read-only comparison, not PARAM_SET/recovery).
    bind_recording MUST suppress all pose writes at stop_at_ns, including a
    pump that crosses the boundary. stop_pose_writes disables poses only;
    HOME/stop-only monitoring stays serviced until close_owned.
    logger_io is the existing static tool I/O, plus read_sensor_selection().
    That method returns original read-only evidence, not a reviewed approval.
    Its pump services the existing owner and USB without competing readers.
    """
    def __init__(self, owner, logger_io, session, *, state, logger_report):
        if type(session) is not MaintenanceRecordingSession:
            raise TypeError('Existing recording session required')
        self.owner, self.io, self.session = owner, logger_io, session
        self.state, self.logger_report = state, logger_report
        self.used = False
        self.logger_start_attempted = False
        self.last_ns = None
        self.report = dict(phases=[], errors=[], cleanup_errors=[],
            observation_window_completed=False, injected_cycle_completed=False,
            data_quality_verified=False, hardware_verified=False,
            physical_resource_factory_implemented=False, parameter_writes=0,
            automatic_retries=0, selection_snapshots={}, record=None)

    def now(self):
        now = logs.now_ns(self.io)
        if type(now) is not int or now < 0 or self.last_ns is not None and now < self.last_ns:
            raise RuntimeError('Maintenance host clock regressed')
        self.last_ns = now
        return now

    def phase(self, name):
        self.report['phases'].append(dict(name=name, at_ns=self.now()))

    def cleanup_phase(self, name):
        # A broken clock must not suppress shutdown. Preserve an UNKNOWN
        # timestamp, not a forged monotonic time, and keep the result failed.
        try:
            self.phase(name)
        except BaseException as exc:
            self.report['phases'].append(dict(name=name, at_ns=None))
            self.report['cleanup_errors'].append('phase_clock: '+str(exc))

    def original_observation(self):
        now = self.now()
        obs = self.owner.observation(now)
        if type(obs) is not MaintenanceObservation or obs.now_ns != now:
            raise RuntimeError('Exact current assessment of original owner evidence required')
        # All original receipts remain in obs; do NOT replace them with now.
        return obs

    def snapshot_selection(self, name):
        row = self.io.read_sensor_selection()
        if not isinstance(row, dict) or not row.get('raw_reply'):
            raise RuntimeError('Original read-only sensor-selection reply required')
        self.report['selection_snapshots'][name] = deepcopy(row)
        # Bind to the exported ULog and review using the existing strict
        # importer AFTER acquisition. Do not invent FC bounds or approve here.

    def _wait_home(self):
        self.phase('WAIT_HOME')
        deadline = self.now()+HOME_WAIT_NS
        home = self.session.gate.home
        while not home.verified:
            if home.aborted or self.now() >= deadline:
                raise RuntimeError('Current manual HOME proof aborted or timed out')
            self.io.pump()
            if not self.io.safe():
                raise RuntimeError('Identity/unarmed proof lost while waiting for HOME')
        if home.aborted:
            raise RuntimeError('Manual HOME fault remains latched')

    def _stop_inputs_and_callbacks(self):
        # Latch the recording on any cleanup failure BEFORE later callbacks
        # could write. Always attempt remaining owned-resource cleanup.
        for name in ('stop_pose_writes', 'stop_inputs', 'detach_recording'):
            try:
                getattr(self.owner, name)()
            except BaseException as exc:
                self.session.abort('maintenance_cleanup:'+name)
                self.report['cleanup_errors'].append(name+': '+str(exc))
        try:
            self.report['record'] = self.session.close()
        except BaseException as exc:
            self.session.abort('recording_close_failed')
            self.report['cleanup_errors'].append('recording_close: '+str(exc))

    def run_injected(self, run_directory):
        """Run with supplied owners; never opens or selects a device.

        Both fixtures and the explicit joint resource factory use this method.
        COMPLETE means lifecycle exercised, NOT dynamic IMU/Vicon quality or
        any flight/maintenance grant. Raw ULog analysis remains mandatory.
        """
        if self.used:
            raise RuntimeError('One maintenance lifecycle only; no automatic retry')
        self.used = True
        try:
            self.phase('PRECHECK')
            if self.owner.port is self.io.port:
                raise RuntimeError('USB and radio must not have competing owners of one port')
            if self.state.get('on_sent') or self.state.get('off_sent') or self.state.get('stop_attempted'):
                raise RuntimeError('Fresh Logger lifecycle required; no takeover or restart')
            if self.logger_report.get('identity', {}).get('uid2') != self.session.gate.uid:
                raise RuntimeError('Logger and radio aircraft identity mismatch')
            if self.io.boot_session != self.session.gate.boot:
                raise RuntimeError('USB/radio boot binding mismatch')
            if self.owner.verify_configuration() is not True:
                raise RuntimeError('Read-only configuration comparison failed before capture')
            if self.session.gate.start_policy == 'motor_isolated':
                self.phase('WAIT_ISOLATED_UNARMED')
                self.owner.wait_ground_ready()
            else:
                self._wait_home()
            # The original USB Logger writer needs a <500ms native unarmed
            # snapshot at logger-on. Do not reuse one from BEFORE a human
            # HOME wait. Same precheck, moved rather than relaxed/repeated.
            logs.prepare_logger(self.io, self.state, self.logger_report)
            self.snapshot_selection('before')
            self.phase('LOGGER_STARTING')
            self.logger_start_attempted = True
            logs.start_logger(self.io, self.state, self.logger_report)
            logs.transition(self.io, self.state, self.logger_report, 'RECORDING')
            # Real adapters need IMU pre-roll and a sensor-selection bracket
            # preceding the first visual sample by >=500ms. Injected owners
            # without this hook retain their established behavior.
            prepare = getattr(self.owner, 'prepare_recording', None)
            if prepare is not None:
                prepare(self.session.gate)
            obs = self.original_observation()
            if not self.session.begin(obs):
                raise RuntimeError(self.session.gate.fault or 'Maintenance begin rejected')
            started = self.now()
            # Never extend/restart the gate's actual window after begin.
            deadline = self.session.gate.started_ns+WINDOW_NS
            self.report.update(capture_started_ns=self.session.gate.started_ns,
                               capture_deadline_ns=deadline)
            if started >= deadline:
                raise RuntimeError('Maintenance window expired before owner binding')
            self.owner.bind_recording(self.session, stop_at_ns=deadline)
            self.phase('CAPTURING')
            while self.now() < deadline:
                if not self.session.gate.check(self.original_observation()):
                    raise RuntimeError(self.session.gate.fault or 'Maintenance proof lost')
                self.io.pump()
                # Planned end is not a new state proof; owner already stops
                # TX at the exact deadline, even inside this pump.
                if self.now() < deadline and not self.io.safe():
                    raise RuntimeError('Logger identity/unarmed proof lost')
            self.report.update(capture_ended_ns=self.now(), observation_window_completed=True)
        except BaseException as exc:
            self.session.abort(type(exc).__name__+': '+str(exc))
            self.report['errors'].append(type(exc).__name__+': '+str(exc))
        finally:
            self.cleanup_phase('QUIESCING')
            self._stop_inputs_and_callbacks()
            try:
                if (self.logger_start_attempted and self.state.get('on_sent')
                        and not self.state.get('stop_attempted')):
                    self.cleanup_phase('LOGGER_STOPPING')
                    logs.stop_and_finalize(self.io, self.state, self.logger_report, self.io.reader)
                if (self.logger_start_attempted and self.state.get('on_sent')
                        and not self.logger_report.get('file_closed_verified')):
                    self.report['keep_USB_power_required'] = True
                if self.logger_start_attempted and self.logger_report.get('file_closed_verified'):
                    try:
                        self.snapshot_selection('after')
                    except BaseException as exc:
                        # Loss of radio clock/selection evidence invalidates
                        # calibration, but must not discard a safely closed raw
                        # ULog that can explain the failure.
                        self.report['cleanup_errors'].append('selection_after: '+str(exc))
                    if self.logger_report.get('file_size_stable'):
                        self.cleanup_phase('EXPORTING')
                        destination = logs.export_closed_ulog(self.io, self.state, self.logger_report,
                                                              run_directory)
                        framing = logs.scan_records(destination.read_bytes())
                        self.report['ULog_framing'] = framing
                        if not framing['ok']:
                            raise RuntimeError('Exported ULog framing: '+framing['reason'])
                if self.logger_start_attempted and self.owner.verify_configuration() is not True:
                    raise RuntimeError('Read-only configuration changed after capture; no restoration')
            except BaseException as exc:
                self.report['cleanup_errors'].append('Logger/export: '+type(exc).__name__+': '+str(exc))
                if (self.logger_start_attempted and self.state.get('on_sent')
                        and not self.logger_report.get('file_closed_verified')):
                    self.report['keep_USB_power_required'] = True
            finally:
                # FileReader/USB finalizer is reused, not a new FTP owner.
                try:
                    logs.finalize_resources(self.io.port, self.io.reader, self.logger_report)
                except BaseException as exc:
                    self.report['cleanup_errors'].append('Logger_finalizer: '+str(exc))
                try:
                    if self.owner.port is not self.io.port:
                        self.owner.close_owned()
                except BaseException as exc:
                    self.report['cleanup_errors'].append('owner_close: '+str(exc))
                if not self.logger_report.get('serial_closed'):
                    self.report['cleanup_errors'].append('USB_serial_not_closed')
                if getattr(self.owner.port, 'is_open', True):
                    self.report['cleanup_errors'].append('radio_transport_not_closed')
                if self.logger_report.get('ftp_close_error') or self.logger_report.get('serial_close_error'):
                    self.report['cleanup_errors'].append('Logger_resource_close_failed')
            try:
                # Copy/export only AFTER callbacks and owned transports stop.
                # Retain failed partial runs too; do not recalibrate/approve.
                self.report['capture_trace'] = self.session.recorder.export()
            except BaseException as exc:
                self.report['cleanup_errors'].append('RAM_trace_export: '+str(exc))
            record = self.report['record'] or {}
            self.report['injected_cycle_completed'] = bool(
                self.report['observation_window_completed'] and record.get('callbacks_closed_cleanly')
                and self.logger_report.get('exported') and self.report.get('ULog_framing', {}).get('ok')
                and not self.report['errors'] and not self.report['cleanup_errors'])
            self.cleanup_phase('COMPLETE' if self.report['injected_cycle_completed'] else 'FAILED')
            if self.report['cleanup_errors']:
                self.report['injected_cycle_completed'] = False
                self.report['phases'][-1]['name'] = 'FAILED'
        return result(**deepcopy(self.report), logger=deepcopy(self.logger_report))
