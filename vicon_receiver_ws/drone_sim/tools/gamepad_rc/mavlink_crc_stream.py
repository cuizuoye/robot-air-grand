"""Unsigned maintenance RX parser: strict CRC, byte-wise error resynchronization.

Never reconstructs damaged bytes. On an invalid candidate, discard only its
first byte, not its claimed length (which may overlap an intact next frame).
All failures remain BAD_DATA for the existing transaction/readiness guards.
Definitions are pinned to the locally built MicoAir firmware; unknown schemas
or signed frames fail closed. This module has no serial/network/control API.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path

from pymavlink.dialects.v20 import common as m
from pymavlink.generator.mavcrc import x25crc

DEFINITIONS=Path(__file__).with_name('micoair_crc_definitions.json')


class CrcResyncParser:
    def __init__(self,definitions=DEFINITIONS):
        if m.MAVLINK_IGNORE_CRC:raise ValueError('MAVLINK_IGNORE_CRC must remain disabled')
        raw=Path(definitions).read_bytes();record=json.loads(raw)
        if record.get('schema')!=1 or not record.get('messages'):
            raise ValueError('Pinned firmware CRC definitions required')
        self.table={int(k):tuple(v) for k,v in record['messages'].items()}
        for mid,entry in self.table.items():
            if (not 0<=mid<2**24 or len(entry)!=3 or any(type(v) is not int for v in entry)
                    or not 0<=entry[0]<=255 or not 0<=entry[1]<=entry[2]<=255):
                raise ValueError('Invalid pinned CRC definition')
            if mid in m.mavlink_map and m.mavlink_map[mid].crc_extra!=entry[0]:
                raise ValueError('Host/firmware CRC_EXTRA mismatch: '+str(mid))
        self.definitions_sha256=hashlib.sha256(raw).hexdigest()
        self.buffer=bytearray();self.decoder=m.MAVLink(None)
        self.counts=Counter();self.robust_parsing=True

    def summary(self):
        return {'implementation':'strict_crc_bytewise_resynchronization',
            'definitions_sha256':self.definitions_sha256,'counts':dict(self.counts),
            'pending_bytes':len(self.buffer),'repaired_bytes':0,
            'signed_frames_supported':False,'unknown_schemas_accepted':False}

    def parse_buffer(self,chunk):
        if m.MAVLINK_IGNORE_CRC:raise ValueError('CRC bypass setting changed during session')
        if len(chunk)>1024*1024:raise ValueError('RX chunk exceeds bounded maintenance input')
        self.buffer.extend(chunk);messages=[]

        def reject(reason,count=1,data=None):
            self.counts[reason.split(':',1)[0]]+=1
            messages.append(m.MAVLink_bad_data(bytes(self.buffer[:count]) if data is None else data,reason))
            del self.buffer[:count]

        while self.buffer:
            if self.buffer[0] not in (0xfd,0xfe):
                heads=[i for i in (self.buffer.find(0xfd),self.buffer.find(0xfe)) if i>=0]
                reject('Bad prefix',min(heads) if heads else len(self.buffer));continue
            v2=self.buffer[0]==0xfd;header=10 if v2 else 6
            if len(self.buffer)<header:break
            if v2 and self.buffer[2]:
                reject('Unsupported incompat_flags or signing: unsigned maintenance parser');continue
            length=self.buffer[1]
            mid=int.from_bytes(self.buffer[7:10],'little') if v2 else self.buffer[5]
            sender=self.buffer[5:7] if v2 else self.buffer[3:5]
            if 0 in sender:reject('Invalid source ID');continue
            if mid not in self.table:reject('Unknown CRC schema: '+str(mid));continue
            extra,lo,hi=self.table[mid]
            if length>hi or (not v2 and length!=lo):
                reject('Invalid firmware payload length: '+str(mid));continue
            total=header+length+2
            if len(self.buffer)<total:break
            wire=bytes(self.buffer[:total]);checksum=x25crc(wire[1:-2]);checksum.accumulate(bytes([extra]))
            received=int.from_bytes(wire[-2:],'little')
            if checksum.crc!=received:
                reject(f'invalid MAVLink CRC: msgID {mid} received={received:04x} expected={checksum.crc:04x}',data=wire)
                continue  # Full candidate retained in diagnostic, only ONE byte consumed.
            try:message=self.decoder.decode(bytearray(wire))
            except m.MAVError as exc:
                reject('Decode rejected: '+str(exc),data=wire);continue
            del self.buffer[:total]
            self.counts['crc_valid_frames']+=1;messages.append(message)
        return messages
