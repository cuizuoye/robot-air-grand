"""One static Logger capture's exact command/readonly file-transfer gate."""
from pathlib import PurePosixPath
import re
from mavlink_readonly_probe import readonly_message
from fault_archive_service import HEADER

READ_COMMANDS = (b'mount\n', b'logger status\n', b'listener actuator_armed -n 1\n', b'ps\n')

def valid_log_path(path):
    if not isinstance(path, str) or not re.fullmatch(r'/fs/microsd/log/[A-Za-z0-9_./-]+\.ulg', path):
        return False
    return '..' not in PurePosixPath(path).parts and len(path.encode()) <= 200

def allowed(msg, *, safe, phase, on_sent, off_sent, writer_pids=(), logfile=None):
    kind = msg.get_type()
    if kind in ('HEARTBEAT', 'COMMAND_LONG'):
        return readonly_message(msg)
    if kind == 'PARAM_REQUEST_READ':
        name = msg.param_id.decode() if isinstance(msg.param_id, bytes) else msg.param_id
        return safe and name.rstrip('\0') in ('SDLOG_PROFILE', 'SDLOG_MODE') and (
            msg.target_system, msg.target_component, msg.param_index) == (1, 1, -1)
    if kind == 'SERIAL_CONTROL':
        if not safe or (msg.device, msg.flags, msg.timeout, msg.baudrate) != (10, 2, 0, 0):
            return False
        permitted = (b'', *READ_COMMANDS, *(f'cat /proc/{p}/group/fd\n'.encode() for p in writer_pids))
        if phase == 'STARTING' and not on_sent and not off_sent:
            permitted += (b'logger on\n',)
        if phase in ('RECORDING', 'STOPPING', 'ABORTING') and on_sent and not off_sent:
            permitted += (b'logger off\n',)
        return any(msg.count == len(p) and bytes(msg.data) == p.ljust(70, b'\0') for p in permitted)
    if kind == 'FILE_TRANSFER_PROTOCOL':
        if not safe or not valid_log_path(logfile) or (msg.target_network, msg.target_system, msg.target_component) != (0, 1, 1):
            return False
        raw = bytes(msg.payload)
        _, session, op, size, req, burst, padding, offset = HEADER.unpack_from(raw)
        if session or req or burst or padding or size > 239:
            return False
        if op in (4, 14):
            return offset == 0 and raw[12:12+size] == logfile.encode() and size == len(logfile.encode())
        if op == 5:
            return phase == 'VERIFYING' and 1 <= size <= 239 and 0 <= offset < 16*1024*1024 and not any(raw[12:])
        if op == 1:
            return offset == 0 and size == 0 and not any(raw[12:])
    return False

def parse_writer_pids(text):
    found = []
    for line in text.splitlines():
        if re.search(r'\blog_writer_file\b', line):
            match = re.match(r'\s*(\d+)\s+', line)
            if match and 0 < int(match.group(1)) < 65536:
                found.append(int(match.group(1)))
    if len(set(found)) != 1:
        raise RuntimeError('Exactly one live log_writer_file PID must be readable')
    return tuple(set(found))

def parse_fd_table(text):
    """FAT may omit PATH: retain the descriptor, flags and offset regardless."""
    if not re.search(r'FD\s+POS\s+OFLAGS\s+PATH', text):
        raise RuntimeError('Procfs file descriptor table unavailable or incomplete')
    descriptors = {}
    for line in text.splitlines():
        match = re.match(r'^\s*(\d+)[ \t]+(-?\d+)[ \t]+([0-9a-fA-F]+)(?:[ \t]+([^\r\n]*))?$', line)
        if match:
            fd, position, flags, path = match.groups()
            if int(fd) in descriptors:
                raise RuntimeError('Duplicate descriptor in writer table')
            descriptors[int(fd)] = dict(position=int(position), flags=int(flags, 16), path=(path or '').strip())
        elif re.match(r'^\s*\d+[ \t]', line):
            raise RuntimeError('Malformed descriptor row; do not assume closed')
    if not descriptors:
        raise RuntimeError('Cannot independently inspect writer file descriptors')
    return descriptors


def fd_contains_log(text, logfile):
    """Named positive evidence only. False is NOT evidence that FAT closed it."""
    return any(v['path'] == logfile for v in parse_fd_table(text).values())


def writer_open_descriptors(current, baseline, logfile):
    if not set(baseline).issubset(current):
        raise RuntimeError('Writer baseline descriptors changed during start')
    added = {k: v for k, v in current.items() if k not in baseline}
    if not added or any(not (v['flags'] & 2) or (v['path'] and v['path'] != logfile)
                        for v in added.values()):
        raise RuntimeError('No unambiguous newly opened writable log descriptor')
    return sorted(added)


def writer_returned_to_baseline(current, baseline):
    return (set(current) == set(baseline) and all(
        (current[k]['flags'], current[k]['path']) == (v['flags'], v['path']) for k, v in baseline.items()))
