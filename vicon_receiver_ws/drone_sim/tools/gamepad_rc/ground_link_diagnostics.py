"""Bounded RAM-only timing observer and OFFLINE replay, not a safety gate.

No serial, sender, file writer or background worker. A future authorized owner
may call parse_chunk around its existing single-reader parser. It MUST NOT add
a second recv_match/serial reader. Dump export() only AFTER the round closes.
read_end_ns is userspace observation, NOT electrical UART/kernel arrival time.
"""
from collections import Counter, defaultdict
from copy import deepcopy
import math
import time

from ekf_ground_acceptance import result

ATTITUDE_LIMIT_NS = 500_000_000
RTT_LIMIT_NS = 100_000_000
STREAMS = ('ATTITUDE_QUATERNION', 'HEARTBEAT', 'OUTLOCK1', 'VICON', 'EKF_FUSION')
KINDS = {'read', 'parse', 'frame', 'stream', 'timesync_request', 'timesync_reply',
         'clock_expired', 'safety_revoke', 'round_fault', 'application_update'}


def ns(value):
    if type(value) is not int or value < 0:
        raise ValueError('Original nonnegative monotonic_ns integer required')
    return value


class TimingRecorder:
    """Single-owner callbacks; bounded by count. Saturation is explicit failure."""
    _allowed_kinds = KINDS
    def __init__(self, session, *, capacity=40000, clock=time.monotonic_ns):
        if not isinstance(session, str) or not session or type(capacity) is not int or capacity < 1:
            raise ValueError('Host clock session and positive event capacity required')
        self.session, self.capacity, self.clock = session, capacity, clock
        self.events, self.dropped = [], 0
        self.last = -1
        self.fault = None

    def record(self, kind, *, at_ns, **fields):
        ns(at_ns)
        if kind not in self._allowed_kinds or at_ns < self.last:
            raise ValueError('Known event and nonregressing observation time required')
        if (len(fields)>24 or any(type(v) not in (str,int,float,bool,type(None))
                or isinstance(v,str) and len(v)>512
                or type(v) is float and not math.isfinite(v) for v in fields.values())):
            raise ValueError('Bounded scalar diagnostic fields only; no mutable/raw payloads')
        self.last = at_ns
        if len(self.events) >= self.capacity:
            self.dropped += 1
            self.fault = self.fault or 'diagnostic_capacity_exceeded'
            return False
        # No formatting, serialization, flush or fsync in this callback.
        self.events.append(dict(kind=kind, at_ns=at_ns, **fields))
        if kind in ('round_fault', 'safety_revoke'):
            self.fault = self.fault or fields.get('reason', 'unspecified_fault')
        return True

    def parse_chunk(self, parser, raw, *, read_start_ns, read_end_ns, in_waiting=None):
        """Pure adapter; raw bytes already obtained by the EXISTING reader.

        End-of-read is an upper bound for all bytes in a batch. Frame completion
        may span reads. Application must record original source stamps and its
        own accepted updates separately; decoding never implies state refresh.
        """
        if ns(read_start_ns) > ns(read_end_ns):
            raise ValueError('Read clock regression')
        self.record('read', at_ns=read_end_ns, start_ns=read_start_ns,
                    bytes=len(raw), in_waiting=in_waiting)
        start = ns(self.clock())
        if start < read_end_ns:
            raise ValueError('Parse cannot precede read')
        messages = parser.parse_buffer(raw)
        end = ns(self.clock())
        if end < start:
            raise ValueError('Parse clock regression')
        batch = len(self.events)
        self.record('parse', at_ns=end, start_ns=start, read_end_ns=read_end_ns, batch=batch)
        for msg in messages or []:
            bad = msg.get_type() == 'BAD_DATA'
            self.record('frame', at_ns=end, batch=batch, valid=not bad,
                crc_error=bad and 'invalid MAVLink CRC' in str(getattr(msg, 'reason', '')),
                message_type=msg.get_type(), read_end_ns=read_end_ns,
                system=msg.get_srcSystem(), component=msg.get_srcComponent(),
                seq=msg.get_seq(), name=getattr(msg,'name',None),
                time_boot_ms=getattr(msg,'time_boot_ms',None),
                tc1=getattr(msg,'tc1',None),ts1=getattr(msg,'ts1',None))
        return messages

    def export(self):
        return dict(schema=1, host_clock_session=self.session, events=deepcopy(self.events),
                    dropped_events=self.dropped, recorder_fault=self.fault,
                    physical_authorization=False, flight_authorized=False)


def _distribution(values):
    ordered = sorted(values)
    if not ordered:
        return dict(count=0, max_ns=None, p99_ns=None, mean_ns=None)
    return dict(count=len(ordered), max_ns=ordered[-1],
                p99_ns=ordered[max(0, math.ceil(.99*len(ordered))-1)],
                mean_ns=sum(ordered)/len(ordered))


def summarize(trace, *, start_ns, end_ns):
    """Analyze exactly one supplied window; not permission to extend active20s.

    A 60s receive-only window differs from active fusion. Lack of observations
    fails completeness; it must never be presented as a zero-error healthy run.
    """
    if ns(end_ns) <= ns(start_ns):
        raise ValueError('Positive observation window required')
    events = trace.get('events')
    if not isinstance(events, list) or not trace.get('host_clock_session'):
        raise ValueError('Bounded timing trace with one host clock session required')
    streams, sources, frame_arrivals = defaultdict(list), {}, defaultdict(list)
    intervals = defaultdict(list)
    counts, reasons, replies = Counter(), Counter(), Counter()
    pending, valid_ref, references = {}, None, []
    last = -1
    app_delay, read_time, parse_time, wait_time = [], [], [], []
    gap_rows, ignored_sources = [], 0
    previous_seq = {}
    for e in events:
        at = ns(e.get('at_ns'))
        if at < last or e.get('kind') not in KINDS:
            raise ValueError('Invalid/out-of-order timing trace')
        last = at
        # Preserve ORIGINAL request context crossing the analysis boundary.
        # Do not invent a request at start_ns or count pre-window traffic.
        if at < start_ns:
            if e['kind'] == 'timesync_request':
                token=ns(e.get('token'))
                if token != at:
                    raise ValueError('Original request token/time required')
                pending[token]=at
            elif e['kind'] == 'timesync_reply':
                sent=pending.pop(ns(e.get('token')),None)
                if e.get('decision') == 'accepted':
                    if sent is None or not 0 <= at-sent <= RTT_LIMIT_NS:
                        raise ValueError('Invalid pre-window reference')
                    valid_ref=at
                    references[:]=[at]
            continue
        if at > end_ns:
            continue
        kind = e['kind']
        counts[kind] += 1
        if kind in ('read', 'parse'):
            t = ns(e.get('start_ns'))
            if t > at:
                raise ValueError('Duration clock regression')
            (read_time if kind == 'read' else parse_time).append(at-t)
            if kind == 'read':
                if type(e.get('bytes')) is not int or e['bytes'] < 0:
                    raise ValueError('Invalid read byte count')
                counts['read_bytes'] += e['bytes']
                waiting = e.get('in_waiting')
                if type(waiting) is int and waiting >= 0:
                    counts['in_waiting_max'] = max(counts['in_waiting_max'], waiting)
            else:
                arrival = ns(e.get('read_end_ns'))
                if arrival > t:
                    raise ValueError('Parse precedes read completion')
                wait_time.append(t-arrival)
        elif kind == 'frame':
            counts['valid_frames'] += e.get('valid') is True
            counts['crc_errors'] += e.get('crc_error') is True
            if e.get('valid') is True:
                key = (e.get('system'), e.get('component'))
                if 'read_end_ns' in e:
                    arrival=ns(e['read_end_ns'])
                    if arrival > at:
                        raise ValueError('Frame precedes read completion')
                    if key == (1,1):
                        name=e.get('message_type')
                        if name == 'NAMED_VALUE_INT' and e.get('name') == 'OUTLOCK1':
                            name='OUTLOCK1'
                        if name:
                            frame_arrivals[name].append(arrival)
                seq = e.get('seq')
                if type(seq) is int and 0 <= seq < 256:
                    if key in previous_seq:
                        delta = (seq-previous_seq[key]) % 256
                        counts['sequence_gap_candidates'] += 1 < delta < 128
                    previous_seq[key] = seq
        elif kind == 'stream':
            name = e.get('stream')
            if name not in STREAMS:
                raise ValueError('Unknown stream')
            # Caller must report accepted current identity/source publications,
            # not refresh age with duplicate/cached/CRC-bad frames.
            if e.get('accepted') is not True or (name != 'VICON' and e.get('identity_verified') is not True):
                ignored_sources += 1
                continue
            stamp = ns(e.get('source_ns'))
            if name in sources and stamp <= sources[name]:
                ignored_sources += 1
                continue
            sources[name] = stamp
            if streams[name]:
                intervals[name].append(at-streams[name][-1])
            streams[name].append(at)
        elif kind == 'timesync_request':
            token = ns(e.get('token'))
            if token in pending or token != at:
                raise ValueError('TIMESYNC original request token/time required')
            pending[token] = at
        elif kind == 'timesync_reply':
            token, decision = ns(e.get('token')), e.get('decision')
            if not isinstance(decision, str) or not decision:
                raise ValueError('Existing clock gate decision required')
            replies[decision] += 1
            sent = pending.pop(token, None)
            if sent is not None:
                rtt = at-sent
                counts['timesync_rtt_over_100ms'] += rtt > RTT_LIMIT_NS
                counts['late_replies_rejected'] += rtt > RTT_LIMIT_NS and decision != 'accepted'
                counts['timesync_matched_replies'] += 1
                intervals['TIMESYNC_RTT'].append(rtt)
                if decision == 'accepted':
                    if rtt > RTT_LIMIT_NS:
                        raise ValueError('Trace claims accepted RTT outside existing gate')
                    valid_ref = at
                    references.append(at)
            elif decision == 'accepted':
                raise ValueError('Accepted reply has no original request')
        elif kind == 'clock_expired':
            # Record the real gate transition, not every polling iteration.
            counts['timesync_reference_expiry_events'] += 1
            if valid_ref is not None and at-valid_ref < ATTITUDE_LIMIT_NS:
                raise ValueError('Claimed reference expiry before existing500ms')
        elif kind in ('safety_revoke', 'round_fault'):
            if not isinstance(e.get('reason'), str) or not e['reason']:
                raise ValueError('Fault reason required')
            reasons[e['reason']] += 1
        elif kind == 'application_update':
            parsed = ns(e.get('parse_end_ns'))
            if parsed > at:
                raise ValueError('Application update precedes parser')
            app_delay.append(at-parsed)
    statistics = {}
    for name in STREAMS:
        times = streams[name]
        gaps = intervals[name]
        boundary = [times[0]-start_ns, end_ns-times[-1]] if times else [end_ns-start_ns]
        statistics[name] = dict(samples=len(times), interarrival=_distribution(gaps),
            max_observed_age_ns=max(gaps+boundary),
            observed_hz=(len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 else None,
            leading_or_trailing_gaps_ns=boundary)
        if name == 'ATTITUDE_QUATERNION':
            points = [start_ns]+times+[end_ns]
            gap_rows = [dict(start_ns=a, end_ns=b, gap_ns=b-a)
                        for a,b in zip(points, points[1:]) if b-a > ATTITUDE_LIMIT_NS]
    missing = [n for n in STREAMS if not streams[n]]
    required = ('read', 'parse', 'frame', 'timesync_request', 'timesync_reply', 'application_update')
    missing += [n for n in required if not counts[n]]
    if not references:
        missing.append('accepted_timesync_reference')
    clock_points = ([start_ns] if not references or references[0]>=start_ns else [])+references+[end_ns]
    clock_gaps = [dict(start_ns=a,end_ns=b,gap_ns=b-a)
                  for a,b in zip(clock_points,clock_points[1:]) if b-a >= ATTITUDE_LIMIT_NS]
    complete = not trace.get('dropped_events', 0) and not missing
    fault = bool(reasons or trace.get('recorder_fault') or gap_rows
                 or clock_gaps or counts['timesync_reference_expiry_events'])
    return result(review_complete=complete, timing_window_clear=complete and not fault,
        window_ns=[start_ns,end_ns], streams=statistics, counts=dict(counts),
        attitude_gaps_over_500ms=gap_rows, timesync_decisions=dict(replies),
        timesync_reference_uncovered_intervals=clock_gaps,
        unanswered_request_tokens=list(pending),
        timesync_rtt=_distribution(intervals['TIMESYNC_RTT']),
        read_duration=_distribution(read_time), parse_duration=_distribution(parse_time),
        read_to_parse_wait=_distribution(wait_time), parse_to_application=_distribution(app_delay),
        rejected_or_cached_updates=ignored_sources, missing_evidence=missing,
        authorization_revocation_reasons=dict(reasons), dropped_events=trace.get('dropped_events',0),
        retry_permitted=False, uart_arrival_measured=False,
        frame_read_completion_intervals={n:_distribution([b-a for a,b in zip(ts,ts[1:])])
                                         for n,ts in frame_arrivals.items()},
        data_loss_proven=False, note='Gaps/sequence gaps are not proof of packet loss; read_end is a userspace upper bound')
