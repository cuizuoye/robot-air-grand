"""Owned domain-83 FrameSample input; shared domain-0 receiver is untouched.

Reuses the pinned receiver; process/library provenance is checked once at startup.
No static recapture, independent frame-number join or ROS initialization on import.
"""
import os
from pathlib import Path
import signal
import subprocess
import time

from ev_delay_sources import frame_sample_fields
from vicon_frame_sample_capture import (BINARY, TOPIC, DOMAIN, RuntimeAuditWorker,
    discover_shared_receiver)
from vicon_metadata_audit_process import MetadataAuditProcess
from vicon_receiver_provenance import child_environment, recheck_before_spawn

METADATA_TIMEOUT_NS = 1_000_000_000


class ViconSource:
    def __init__(self, run_dir, provenance, *, latest_only=False, subject='MircoAir'):
        if subject not in ('MircoAir','car01'):raise ValueError('known Vicon subject required')
        self.subject=subject
        self.latest_only = latest_only
        self.subscription_depth = 1 if latest_only else 64
        self.sdk_buffer_size = 1 if latest_only else 200
        self.latest = None
        self.on_frame = lambda row, at: None
        self.count = 0
        self.delivered_count = 0
        self.proc = self.metadata = self.worker = self.node = self.context = self.executor = None
        self.stopped = False
        self.report = dict(provenance=provenance, audits=[], console='', shared_modified=False,
            delivery_policy='latest_only' if latest_only else 'capture_history',
            sdk_buffer_size=self.sdk_buffer_size,
            metadata_audit_scope='startup_only',metadata_timeout_ns=METADATA_TIMEOUT_NS,
            metadata_evidence_dir=str(Path(run_dir).resolve()/'metadata_evidence'))
        try:
            self._open(Path(run_dir), provenance)
        except BaseException:
            self.close()
            raise

    def _open(self, run_dir, provenance):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from vicon_receiver.msg import FrameSample
        shared = discover_shared_receiver()
        self.report['shared_before'] = shared
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=DOMAIN)
        self.node = Node('b3_joint_FrameSample_record', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        others = [(n, ns) for n, ns in self.node.get_node_names_and_namespaces() if n != self.node.get_name()]
        if others: raise RuntimeError('Dedicated domain 83 already occupied: '+str(others))
        # At 330 Hz the generic sensor-data depth 5 covers only 15 ms.
        # Calibration preserves history; flight keeps the newest sample rather
        # than replaying queued motion after a delayed reactor turn. Neither
        # policy changes timestamps or relaxes the downstream freshness gate.
        qos = QoSProfile(depth=self.subscription_depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.node.create_subscription(FrameSample, TOPIC, self.receive, qos)
        self.report['subscription_depth'] = self.subscription_depth
        command = [str(BINARY), '--ros-args', '-r', '__node:=b3_vicon_isolated_receiver',
            '-p', 'hostname:=10.10.10.2', '-p', 'namespace:=vicon_b3_check',
            '-p', 'capture_frame_samples:=true', '-p', 'capture_subject:='+self.subject,
            '-p', 'capture_segment:='+self.subject, '-p', 'stream_mode:=ServerPush',
            '-p', f'buffer_size:={self.sdk_buffer_size}', '-p', 'world_frame:=map', '-p', 'vicon_frame:=vicon_b3_check',
            '-r', '/tf:=/vicon_b3_check/tf', '-r', '/tf_static:=/vicon_b3_check/tf_static',
            '--log-level', 'warn']
        env = child_environment(os.environ, provenance['policy'])
        log_dir = run_dir/'ros_logs'
        log_dir.mkdir(mode=0o700)
        env['ROS_LOG_DIR'] = str(log_dir)
        recheck_before_spawn(provenance)
        self.proc = subprocess.Popen(command, env=env, cwd=run_dir, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, start_new_session=True)
        if Path(os.readlink(f'/proc/{self.proc.pid}/exe')).resolve() != BINARY.resolve():
            raise RuntimeError('Incorrect receiver executable')
        os.set_blocking(self.proc.stdout.fileno(), False)
        self.report.update(child_pid=self.proc.pid, command=command)
        self.metadata = MetadataAuditProcess(self.proc.pid, provenance, shared,
            startup_deadline_ns=time.monotonic_ns()+8_000_000_000,
            request_timeout_ns=METADATA_TIMEOUT_NS,evidence_dir=Path(run_dir)/'metadata_evidence')
        self.worker = RuntimeAuditWorker(self.metadata,timeout_ns=METADATA_TIMEOUT_NS)
        self.startup_audit()

    def receive(self, message):
        at = time.monotonic_ns()
        row = frame_sample_fields(message)
        row.update(callback_ns=at, callback_ros_ns=self.node.get_clock().now().nanoseconds)
        self.latest = row
        self.count += 1
        if not self.latest_only:
            self.on_frame(row, at)

    def accept(self, row):
        if row is None: return
        if len(self.report['audits']) >= 4096: raise RuntimeError('Audit report capacity exceeded')
        self.report['audits'].append(row)
        if row['payload'].get('error'): raise RuntimeError('Vicon provenance: '+row['payload']['error'])

    def startup_audit(self):
        """Complete and close metadata inspection BEFORE serving hardware input."""
        try:
            self.worker.submit(time.monotonic_ns(), dict(
                monitor_shared=self.report['shared_before'] is not None,
                received_rows=self.count, last_callback_ns=None))
            result = self.worker.wait()
            self.report['audit_evidence'] = self.metadata.finish_and_export()
            self.accept(result)
        finally:
            closed = self.worker.close()
            self.report['worker'] = self.worker.status()
            self.worker = self.metadata = None
            if not closed: raise RuntimeError('startup metadata worker did not close')

    def poll(self):
        if self.stopped: return
        if self.proc.poll() is not None: raise RuntimeError('Owned Vicon receiver exited')
        # Drain bursts while preserving a bounded turn for HOME/USB/TRS.
        deadline = time.monotonic_ns()+2_000_000
        for _ in range(64):
            before = self.count
            self.executor.spin_once(timeout_sec=0)
            if self.latest_only and self.count == before: break
            if time.monotonic_ns() >= deadline: break
        if self.latest_only and self.count != self.delivered_count:
            # Record at most the newest consumed frame per control turn. An
            # invalid newest frame also replaces the previous valid one.
            self.on_frame(self.latest, self.latest['callback_ns'])
            self.delivered_count = self.count
        raw = self.proc.stdout.read(4096)
        if raw:
            self.report['console'] += raw.decode('utf8', 'replace')
            excess=max(0,len(self.report['console'])-65536)
            self.report['console_dropped_chars']=self.report.get('console_dropped_chars',0)+excess
            self.report['console']=self.report['console'][-65536:]

    def now_ros_ns(self): return self.node.get_clock().now().nanoseconds

    def stop_inputs(self):
        self.stopped = True
        self.on_frame = lambda row, at: None

    def close(self):
        self.stop_inputs()
        errors = []
        if self.worker:
            try:
                self.accept(self.worker.wait())
                self.report['audit_evidence'] = self.metadata.finish_and_export()
            except BaseException as exc: errors.append(str(exc))
            if not self.worker.close(): errors.append('metadata worker did not close')
            self.report['worker'] = self.worker.status()
            self.worker = None
        elif self.metadata:
            self.metadata.close(deadline_ns=time.monotonic_ns()+METADATA_TIMEOUT_NS)
        if self.proc:
            if self.proc.poll() is None:
                os.killpg(self.proc.pid, signal.SIGINT)
                try: self.proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                    self.proc.wait(timeout=2)
                    errors.append('owned receiver required forced shutdown')
            if self.proc.stdout: self.proc.stdout.close()
            self.report['child_exit'] = self.proc.returncode
            self.proc = None
        if self.executor: self.executor.shutdown(timeout_sec=0); self.executor = None
        if self.node: self.node.destroy_node(); self.node = None
        if self.context: self.context.try_shutdown(); self.context = None
        self.report['cleanup_errors'] = errors
        if errors: raise RuntimeError('; '.join(errors))
