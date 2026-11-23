"""Durable flight-effect journal, NOT a flight authorization or recovery sender."""
from copy import deepcopy
import json
import os
from pathlib import Path

from preparation_authorization import exclusive_json,sync_directory


class FlightExecutionJournal:
    def __init__(self,path,*,boot_session,rigid_revision):
        self.path=Path(path)
        if self.path.name!='flight_recovery.json':
            raise ValueError('Use flight_recovery.json so the common recovery gate can find it')
        if not boot_session or not rigid_revision:raise ValueError('Current boot/revision required')
        self.record=dict(schema=1,scope='single-vertical-flight-effects-v1',
            boot_session=boot_session,rigid_revision=rigid_revision,
            needs_restore=False,needs_operator_review=False,verified_closed=False,events=[])
        self.failed=False;self.closed=False;self.streams=set()
        exclusive_json(self.path,self.record)

    def _save(self):
        pending=Path(str(self.path)+'.pending')
        try:
            exclusive_json(pending,self.record)
            os.replace(pending,self.path)
            sync_directory(self.path.parent)
        except BaseException:
            self.failed=True
            raise  # Preserve any .pending file; never erase uncertain effects.

    def before(self,runtime,record,msg):
        if self.failed or self.closed:raise PermissionError('Journal failed/closed; no new effects')
        if (runtime.inputs.boot_session,runtime.inputs.rigid_revision)!=(
                self.record['boot_session'],self.record['rigid_revision']):
            raise PermissionError('Journal cannot be rebound after a reboot/calibration change')
        kind=msg.get_type()
        # Log the start of each continuous stream once, not an fsync per
        # 32Hz pose. Commands and parameter transactions are always recorded.
        stream=kind if kind in ('VISION_POSITION_ESTIMATE','SET_POSITION_TARGET_LOCAL_NED','RC_CHANNELS_OVERRIDE') else None
        if stream in self.streams:return True
        entry=dict(at=runtime.now,kind=kind,wire_hex=record['wire_hex'],
            phase=runtime.core.phase,baseline=deepcopy(runtime.coordinator.fusion.baseline),
            delivery='unknown_until_confirmed_by_current_fc_state')
        self.record['events'].append(entry)
        self.record.update(needs_restore=True,needs_operator_review=True,verified_closed=False)
        self._save()  # MUST finish before any possibly effective bytes.
        if stream:self.streams.add(stream)
        return True

    def finalize(self,runtime,io):
        if self.failed or io.delivery_uncertain:return False
        if runtime.sync_rates and not runtime.sync_rates.summary()['restoration_verified']:return False
        if not self.record['events']:
            safe=True
        else:
            c=runtime.context()
            safe=bool(runtime.closed and runtime.coordinator.fusion.phase=='CLOSED'
                and c.restore_ground() and runtime.core.phase in ('COMPLETE','BLOCKED','STOP_LATCHED')
                and (runtime.inputs.boot_session,runtime.inputs.rigid_revision)==(
                    self.record['boot_session'],self.record['rigid_revision']))
        if not safe:return False
        self.record.update(needs_restore=False,needs_operator_review=False,verified_closed=True)
        self._save();self.closed=True
        return True

    @staticmethod
    def inspect(path):
        """Read-only recovery status, never sends restores or clears a latch."""
        path=Path(path)
        if path.name!='flight_recovery.json' or not path.is_file():
            raise ValueError('A regular flight_recovery.json file is required; no device/FIFO reads')
        record=json.loads(path.read_text())
        return dict(path=str(path),pending=Path(str(path)+'.pending').exists(),
            needs_restore=record.get('needs_restore') is not False,
            needs_operator_review=record.get('needs_operator_review') is not False,
            verified_closed=record.get('verified_closed') is True,events=len(record.get('events',[])),
            automatic_recovery_available=False)
