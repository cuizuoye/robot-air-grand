#!/usr/bin/env python3
"""USB-only exact ORIGINAL telemetry restoration, never fusion or flight.

Uses the existing rate-trial's two-second disarmed/lock snapshot bound solely
for non-actuating restoration. Does not relax any preparation or flight gate.
Reads a retained journal; never clears it or any stop latch automatically.
"""
import argparse
import json
from pathlib import Path
import time

import serial
from pymavlink.dialects.v20 import common as m
from mavlink_crc_stream import CrcResyncParser
from mavlink_readonly_probe import Evidence,PassivePowerEvidence,FC_USB,readonly_message
from estop_only_policy import LISTENER,parse_actuator_snapshot,shell_message,shell_message_allowed
from persistent_shell import reply_complete
from trs_message_interval_trial import interval_message,CANDIDATES
from trs_uart_readonly_check import UID2
from prepare_drone import TRS,require_port_free
from preparation_authorization import exclusive_json,SCOPE

RESTORABLE_IDS = frozenset(CANDIDATES) | {252}  # Existing joint OUTLOCK1 transaction.


def retained_rates(record):
    rates = record['rates']
    original = {int(k): v for k, v in rates['original_intervals_us'].items()}
    changed = rates['possibly_changed']
    if (record['scope'] != SCOPE or record['needs_restore'] is not True
            or not original or not set(original) <= RESTORABLE_IDS
            or not changed or any(type(k) is not int for k in changed)
            or len(changed) != len(set(changed)) or not set(changed) <= set(original)
            or any(type(v) is not int or not 0 <= v <= 60_000_000 for v in original.values())):
        raise ValueError('Retained exact original rate journal required')
    return original, changed


def allowed(msg,*,safe=False,locked=False,original=None,changed=()):
    if readonly_message(msg):return True
    if msg.get_type()=='SERIAL_CONTROL':return safe and shell_message_allowed(msg)
    if (msg.get_type()!='COMMAND_LONG' or not safe
            or (msg.target_system,msg.target_component,msg.confirmation)!=(1,1,0)
            or any(getattr(msg,'param'+str(i))!=0 for i in range(3,8))):return False
    if msg.command==510:return msg.param1 in RESTORABLE_IDS and msg.param2==0
    key=int(msg.param1)
    return bool(msg.command==511 and locked and msg.param1 == key and key in changed and key in RESTORABLE_IDS
        and original is not None and key in original
        and msg.param2==(original[key] if original[key]>0 else -1))


def run(args):
    if not args.confirm_battery_disconnected or not args.confirm_usb_only_stationary or not FC_USB.exists():
        raise ValueError('Current battery-disconnected USB-only stationary scope required')
    if args.output.exists():raise ValueError('New evidence output required')
    record=json.loads(args.journal.read_text())
    original, changed = retained_rates(record)
    require_port_free(TRS)
    enc=m.MAVLink(None,srcSystem=250,srcComponent=221);dec=CrcResyncParser()
    ev=Evidence(250,221,expected_parameters=());power=PassivePowerEvidence()
    start=time.monotonic();next_hb=next_version=0.;buffer=''
    intervals={};acks=[];snapshots=[];queries=[];tx=[];actual={};error=None
    def elapsed():return time.monotonic()-start
    def safe():
        return bool(FC_USB.exists() and ev.identity and ev.identity['uid2']==UID2
            and ev.identity['firmware']=='1.14.3' and not ev.fatal and not power.reboot_observed
            and ev.heartbeats and not ev.heartbeats[-1]['armed']
            and elapsed()-ev.heartbeats[-1]['elapsed_s']<2)
    def locked():
        return bool(snapshots and snapshots[-1]['lockdown'] and not snapshots[-1]['armed']
            and elapsed()-snapshots[-1]['at']+snapshots[-1]['source_age']<2)
    def send(msg):
        if not allowed(msg,safe=safe(),locked=locked(),original=original,changed=changed):
            raise RuntimeError('Outside exact original-rate restoration scope')
        wire=msg.pack(enc)
        if port.write(wire)!=len(wire):raise OSError('Short write; no automatic retry')
        enc.seq=(enc.seq+1)%256
        tx.append(dict(at=elapsed(),type=msg.get_type(),command=getattr(msg,'command',None),
            param1=getattr(msg,'param1',None),param2=getattr(msg,'param2',None)))
    def pump():
        nonlocal next_hb,next_version,buffer
        t=elapsed()
        if t>35 or not FC_USB.exists():raise RuntimeError('USB-only restoration deadline/presence lost')
        for msg in dec.parse_buffer(port.read(4096)) or ():
            ev.ingest(msg,t);power.ingest(msg,t)
            if (msg.get_srcSystem(),msg.get_srcComponent())!=(1,1):continue
            if msg.get_type()=='MESSAGE_INTERVAL':intervals[int(msg.message_id)]=(t,int(msg.interval_us))
            elif msg.get_type()=='COMMAND_ACK' and msg.command==511 and (msg.target_system,msg.target_component)==(250,221):
                acks.append(dict(at=t,result=msg.result))
            elif msg.get_type()=='SERIAL_CONTROL':buffer+=bytes(msg.data[:msg.count]).decode('utf8','replace')
        if ev.fatal or power.reboot_observed:raise RuntimeError(ev.fatal or 'FC reboot observed')
        if t>=next_hb:
            send(enc.heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0));next_hb=t+1
        if ev.heartbeats and ev.identity is None and t>=next_version:
            ev.version_requested_at=t;send(enc.command_long_encode(1,1,512,0,148,0,0,0,0,0,0));next_version=t+3
        time.sleep(.001)
    def wait(test,seconds):
        end=elapsed()+seconds
        while not test():
            if elapsed()>end:raise RuntimeError('Bounded response timeout')
            pump()
    def check_lock():
        nonlocal buffer
        for _ in range(4):
            buffer='';send(shell_message(enc));deadline=elapsed()+3;poll=elapsed()+.2
            while elapsed()<deadline:
                pump();snap=parse_actuator_snapshot(buffer)
                if snap and reply_complete(buffer,LISTENER):
                    boot=power.boot_samples[-1]['time_boot_ms'] if power.boot_samples else None
                    age=max(0,boot/1000-snap['timestamp_us']/1e6) if boot is not None else None
                    queries.append(dict(parsed=snap,boot_ms=boot,source_age=age,at=elapsed()))
                    if (boot is not None and not snap['armed'] and snap['lockdown']
                            and not snap['manual_lockdown'] and not snap['force_failsafe']
                            and abs(boot/1000-snap['timestamp_us']/1e6)<2):
                        snapshots.append({**snap,'at':elapsed(),'source_age':age});return
                    break
                if elapsed()>=poll:send(shell_message(enc,poll=True));poll=elapsed()+.2
            else:queries.append(dict(at=elapsed(),incomplete_reply=buffer[-800:]))
        raise RuntimeError('No fresh locked disarmed state; no rate SET sent')
    try:
        with serial.Serial(str(TRS),115200,timeout=0,write_timeout=.3,exclusive=True) as port:
            wait(safe,8)
            for key in reversed(changed):
                check_lock();sent=elapsed();send(interval_message(enc,key,original[key] if original[key]>0 else -1,short_status=key==252))
                wait(lambda:any(a['at']>=sent and a['result']==0 for a in acks),3)
                pause=elapsed()+.2
                while elapsed()<pause:pump()
                requested=elapsed();send(interval_message(enc,key,short_status=key==252))
                wait(lambda:key in intervals and intervals[key][0]>=requested,3)
                if intervals[key][1]!=original[key]:raise RuntimeError('Original interval readback mismatch')
            for key in original:
                requested=elapsed();send(interval_message(enc,key,short_status=key==252))
                wait(lambda:key in intervals and intervals[key][0]>=requested,3)
                actual[key]=intervals[key][1]
            if actual!=original:raise RuntimeError('Incomplete exact original-rate readback')
    except (Exception,KeyboardInterrupt) as exc:error=str(exc)
    result=dict(scope=__doc__,journal=str(args.journal),identity=ev.identity,original=original,actual=actual,
        restoration_verified=error is None and actual==original,error=error,snapshots=snapshots,queries=queries,
        acks=acks,tx=tx,parser=dec.summary(),flight_authorized=False,serial_port_released=True)
    exclusive_json(args.output,result)
    print(json.dumps(dict(restoration_verified=result['restoration_verified'],actual=actual,error=error,queries=queries),indent=2))
    return 0 if result['restoration_verified'] else 2


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--journal',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--confirm-battery-disconnected',action='store_true')
    ap.add_argument('--confirm-usb-only-stationary',action='store_true')
    raise SystemExit(run(ap.parse_args()))
