"""One read-only metadata interpreter, not a receiver/ROS/SDK entry point.

The existing bounded metadata thread transports one JSON request/result. CPU
classification runs in this separate interpreter, never sharing the ROS GIL.
No network/socket, ROS initialization, serial, SDK or control calls occur here.
"""
import hashlib
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
import time


MAX_PACKET_BYTES=2_000_000
MAX_SUMMARY_BYTES=4096
AUDIT_NS=100_000_000


def audit_timeout(value):
    if type(value) is not int or not AUDIT_NS <= value <= 1_000_000_000:
        raise ValueError('Metadata timeout must be 100ms..1s')
    return value


def persist_evidence(directory, name, document):
    """Worker-only atomic diagnostic snapshot; never on the ROS receive loop."""
    raw=evidence_bytes(document)
    if len(raw)>MAX_PACKET_BYTES: raise RuntimeError('metadata_evidence_capacity_exceeded')
    path=Path(directory)/name
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_bytes(raw)
    temporary.replace(path)


def evidence_bytes(payload):
    return json.dumps(payload,separators=(',',':'),sort_keys=True,allow_nan=False).encode()


class AuditEvidenceStore:
    """Only latest and first failure, kept in helper RAM, never on disk."""
    def __init__(self):
        self.latest=None; self.first_failure=None; self.frozen=False

    def record(self,serial,payload):
        if self.frozen or self.first_failure is not None:
            raise RuntimeError('metadata_audit_frozen_or_failure_latched')
        raw=evidence_bytes(payload)
        if len(raw)>MAX_PACKET_BYTES: raise RuntimeError('metadata_evidence_capacity_exceeded')
        reference=dict(audit_serial=serial,sha256=hashlib.sha256(raw).hexdigest())
        self.latest=dict(audit_reference=reference,raw=raw)
        if payload.get('error'): self.first_failure=self.latest
        summary=dict(audit_reference=reference,verified=not bool(payload.get('error')),
                     error=payload.get('error'))
        if 'shared_observation' in payload: summary['shared_observation']=payload['shared_observation']
        return summary

    def export(self,reference):
        self.frozen=True
        for record in (self.latest,self.first_failure):
            if record is not None and record['audit_reference']==reference:
                return dict(audit_reference=reference,payload=json.loads(record['raw']))
        raise RuntimeError('metadata_evidence_not_retained')


class MetadataAuditProcess:
    """Private child and bounded pipes; no pool, retries or receiver access.

    Only initial interpreter readiness may wait up to the existing 8s startup
    bound, BEFORE the first observation. Requests default to 100ms; the joint
    collector explicitly uses a 1s metadata budget, not a pose freshness budget.
    The transport itself is called by RuntimeAuditWorker, not the ROS owner.
    """
    def __init__(self,pid,provenance,shared_before,*,startup_deadline_ns=None,
                 request_timeout_ns=AUDIT_NS, evidence_dir=None):
        self.request_timeout_ns=audit_timeout(request_timeout_ns)
        self.evidence_dir=None if evidence_dir is None else Path(evidence_dir).resolve()
        if self.evidence_dir is not None: self.evidence_dir.mkdir(mode=0o700)
        self.last_request=None
        self._closed=threading.Event(); self._buffer=bytearray(); self._serial=0
        self._fault=None; self._forced=False; self._init=dict(
            target_pid=pid,provenance=provenance,shared_before=shared_before)
        if self.evidence_dir is not None: self._init['evidence_dir']=str(self.evidence_dir)
        self._configured=False; self._frozen=False; self._last_reference=None; self._first_failure_reference=None
        self._busy=threading.Lock()
        self.command=[sys.executable,'-B',str(Path(__file__).resolve()),'--metadata-worker']
        self.proc=subprocess.Popen(self.command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,close_fds=True,start_new_session=True)
        try:
            os.set_blocking(self.proc.stdin.fileno(),False)
            os.set_blocking(self.proc.stdout.fileno(),False)
            # Share the capture's existing absolute startup bound. This must
            # not give the receiver an extra eight seconds before first data.
            ready_deadline=min(time.monotonic_ns()+8_000_000_000,
                startup_deadline_ns if startup_deadline_ns is not None else float('inf'))
            ready=self._receive(ready_deadline,capacity=MAX_SUMMARY_BYTES)
            if time.monotonic_ns()>ready_deadline:
                raise RuntimeError('metadata_worker_startup_deadline_missed')
            if ready!=dict(ready=True,pid=self.proc.pid,role='read_only_metadata'):
                raise RuntimeError('metadata_worker_invalid_readiness')
        except BaseException:
            self.close(); raise

    def _receive(self,deadline,*,capacity=MAX_PACKET_BYTES):
        while True:
            end=self._buffer.find(b'\n')
            if end>=0:
                raw=bytes(self._buffer[:end]); del self._buffer[:end+1]
                if self._buffer: raise RuntimeError('metadata_worker_unsolicited_output')
                return json.loads(raw)
            if len(self._buffer)>=capacity:
                raise RuntimeError('metadata_worker_result_capacity_exceeded')
            if self._closed.is_set(): raise RuntimeError('metadata_worker_closed')
            remaining=(deadline-time.monotonic_ns())/1e9
            if remaining<=0: raise RuntimeError('runtime_audit_deadline_missed')
            ready,_,_=select.select([self.proc.stdout],[],[],min(.01,remaining))
            if ready:
                data=os.read(self.proc.stdout.fileno(),min(65536,capacity-len(self._buffer)))
                if not data: raise RuntimeError('metadata_worker_exited_or_pipe_closed')
                self._buffer.extend(data)

    def _send(self,document,deadline):
        raw=(json.dumps(document,separators=(',',':'),allow_nan=False)+'\n').encode()
        if len(raw)>MAX_PACKET_BYTES: raise RuntimeError('metadata_worker_request_capacity_exceeded')
        offset=0
        while offset<len(raw):
            if self._closed.is_set(): raise RuntimeError('metadata_worker_closed')
            remaining=(deadline-time.monotonic_ns())/1e9
            if remaining<=0: raise RuntimeError('runtime_audit_deadline_missed')
            _,ready,_=select.select([],[self.proc.stdin],[],min(.01,remaining))
            if ready:
                try: offset+=os.write(self.proc.stdin.fileno(),raw[offset:offset+65536])
                except BlockingIOError: continue

    def _request(self,kind,*,context=None,reference=None):
        if self._closed.is_set() or self._fault: raise RuntimeError('metadata_worker_closed_or_failed')
        if not self._busy.acquire(blocking=False): raise RuntimeError('metadata_worker_request_already_pending')
        self._serial+=1
        request=dict(serial=self._serial,kind=kind)
        if kind=='audit':
            request['context']=context
            if not self._configured: request['initialize']=self._init
        else: request['audit_reference']=reference
        start=time.monotonic_ns(); deadline=start+self.request_timeout_ns
        self.last_request=dict(serial=self._serial,kind=kind,start_ns=start,deadline_ns=deadline)
        cpu=time.thread_time_ns()
        try:
            self._send(request,deadline)
            result=self._receive(deadline,capacity=MAX_SUMMARY_BYTES if kind=='audit' else MAX_PACKET_BYTES)
            if result.get('serial')!=self._serial or result.get('worker_pid')!=self.proc.pid:
                raise RuntimeError('metadata_worker_result_identity_mismatch')
            if time.monotonic_ns()>deadline: raise RuntimeError('runtime_audit_deadline_missed')
            if result.get('kind')!=('audit_summary' if kind=='audit' else 'audit_evidence'):
                raise RuntimeError('metadata_worker_result_kind_mismatch')
            self._configured=True
            payload=result['payload']
            if kind=='audit':
                ref=payload.get('audit_reference')
                if (not isinstance(ref,dict) or ref.get('audit_serial')!=self._serial or
                    not isinstance(ref.get('sha256'),str) or len(ref['sha256'])!=64 or
                    any(c not in '0123456789abcdef' for c in ref['sha256']) or
                    type(payload.get('verified')) is not bool or
                    payload['verified']!= (not bool(payload.get('error'))) or
                    any(k in payload for k in ('snapshot','classification','loaded'))):
                    raise RuntimeError('metadata_worker_invalid_audit_summary')
                self._last_reference=ref
                if payload.get('error') and self._first_failure_reference is None:
                    self._first_failure_reference=ref
            elif (result.get('audit_reference')!=reference or
                    hashlib.sha256(evidence_bytes(payload)).hexdigest()!=reference['sha256']):
                raise RuntimeError('metadata_evidence_reference_or_hash_mismatch')
            if kind=='export':
                if time.monotonic_ns()>deadline: raise RuntimeError('runtime_audit_deadline_missed')
                return dict(audit_reference=reference,payload=payload)
            payload['metadata_process_timing']=dict(worker_pid=self.proc.pid,
                start_ns=result['start_ns'],end_ns=result['end_ns'],cpu_ns=result['cpu_ns'],
                transport_thread_cpu_ns=time.thread_time_ns()-cpu,
                transport_start_ns=start,transport_end_ns=time.monotonic_ns())
            return payload
        except BaseException as exc:
            self._fault=type(exc).__name__+': '+str(exc); raise
        finally: self._busy.release()

    def __call__(self,context):
        if self._frozen: raise RuntimeError('metadata_audit_frozen_after_observation')
        if self._first_failure_reference is not None: raise RuntimeError('metadata_audit_failure_latched')
        return self._request('audit',context=context)

    def finish_and_export(self):
        """Freeze evidence outside reception: before it starts or after it stops."""
        if self._last_reference is None: raise RuntimeError('metadata_no_audit_evidence')
        self._frozen=True
        references=[self._last_reference]
        if self._first_failure_reference and self._first_failure_reference not in references:
            references.append(self._first_failure_reference)
        return [self._request('export',reference=ref) for ref in references]

    def close(self,*,deadline_ns=None):
        """Stop only this child; a forced stop cannot count as clean success."""
        if self._closed.is_set(): return self.proc.poll()==0 and not self._forced
        deadline=deadline_ns or time.monotonic_ns()+self.request_timeout_ns
        self._closed.set()
        # EOF lets an idle worker leave; a stuck read-only inspector is killed
        # within the configured close budget, never left running after capture.
        self.proc.stdin.close()
        try: self.proc.wait(timeout=max(0,(deadline-time.monotonic_ns())/1e9)*.5)
        except subprocess.TimeoutExpired:
            self._forced=True; self.proc.kill()
            try: self.proc.wait(timeout=max(0,(deadline-time.monotonic_ns())/1e9))
            except subprocess.TimeoutExpired: return False
        finally:
            if self.proc.poll() is not None: self.proc.stdout.close()
        return self.proc.returncode==0 and not self._forced

    def status(self):
        return dict(pid=self.proc.pid,alive=self.proc.poll() is None,returncode=self.proc.returncode,
            closed=self._closed.is_set(),forced_stop=self._forced,fault=self._fault,
            configured=self._configured,requests=self._serial,command=self.command,
            maximum_packet_bytes=MAX_PACKET_BYTES,maximum_active_result_bytes=MAX_SUMMARY_BYTES,
            request_timeout_ns=self.request_timeout_ns,last_request=self.last_request,
            evidence_dir=None if self.evidence_dir is None else str(self.evidence_dir),
            frozen=self._frozen,last_reference=self._last_reference,role='read_only_metadata')


def metadata_worker_main(source=None,sink=None):
    """Importing the collector does not execute its run()/ROS/SDK entry point."""
    from vicon_frame_sample_capture import runtime_audit_snapshot
    source=source or sys.stdin.buffer; sink=sink or sys.stdout.buffer
    def emit(document,capacity=MAX_PACKET_BYTES):
        raw=(json.dumps(document,separators=(',',':'),allow_nan=False)+'\n').encode()
        if len(raw)>capacity: raise RuntimeError('metadata_worker_result_capacity_exceeded')
        sink.write(raw); sink.flush()
    emit(dict(ready=True,pid=os.getpid(),role='read_only_metadata'))
    initialize=None; serial=0; store=AuditEvidenceStore()
    while True:
        line=source.readline(MAX_PACKET_BYTES+1)
        if not line: return
        if len(line)>MAX_PACKET_BYTES or not line.endswith(b'\n'):
            raise RuntimeError('metadata_worker_invalid_request_frame')
        request=json.loads(line)
        if request['serial']!=serial+1: raise RuntimeError('metadata_worker_request_sequence_invalid')
        serial=request['serial']
        if request.get('kind','audit')=='export':
            record=store.export(request['audit_reference'])
            emit(dict(serial=serial,kind='audit_evidence',worker_pid=os.getpid(),**record))
            continue
        if request.get('kind','audit')!='audit': raise RuntimeError('metadata_worker_request_kind_invalid')
        if initialize is None: initialize=request['initialize']
        elif 'initialize' in request: raise RuntimeError('metadata_worker_reinitialization_refused')
        if store.frozen or store.first_failure is not None:
            raise RuntimeError('metadata_audit_frozen_or_failure_latched')
        start=time.monotonic_ns(); cpu=time.thread_time_ns()
        evidence_dir=initialize.get('evidence_dir')
        if evidence_dir is not None:
            persist_evidence(evidence_dir,'request.json',dict(serial=serial,start_ns=start,
                target_pid=initialize['target_pid'],context=request['context']))
        payload=runtime_audit_snapshot(initialize['target_pid'],initialize['provenance'],
            initialize['shared_before'],request['context'],
            **({} if evidence_dir is None else dict(snapshot_path=Path(evidence_dir)/'raw_maps.json')))
        summary=store.record(serial,payload)
        if evidence_dir is not None:
            record=dict(audit_reference=summary['audit_reference'],payload=payload)
            persist_evidence(evidence_dir,'latest_result.json',record)
            if payload.get('error'):persist_evidence(evidence_dir,'first_failure.json',record)
        emit(dict(serial=serial,kind='audit_summary',worker_pid=os.getpid(),payload=summary,start_ns=start,
                  end_ns=time.monotonic_ns(),cpu_ns=time.thread_time_ns()-cpu),capacity=MAX_SUMMARY_BYTES)


if __name__=='__main__':
    if sys.argv[1:]!=['--metadata-worker']: raise SystemExit('metadata worker entry only')
    metadata_worker_main()
    # Clean EOF only: all results were flushed and every inspected fd closed.
    # This helper has no nodes/SDK/sensors/log writer. Do not let unrelated
    # imported-module finalizers delay the configured shutdown budget.
    # Exceptions never reach this path and cannot become a clean returncode.
    os._exit(0)
