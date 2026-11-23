"""Reuse the reviewed existing receiver; no spawn, network changes or controls."""
from pathlib import Path
import os
import sys

PROVENANCE=Path('/home/czy/vicon_receiver_ws/drone_sim/tools/gamepad_rc')
PROC=Path('/proc')


def receiver_inventory(proc_root=None):
    """Read process metadata only; never stop, adopt or edit a foreign domain.

    Unset ROS_DOMAIN_ID is domain0. An unreadable/ambiguous domain for a live
    receiver must not be guessed to be foreign and silently ignored.
    """
    result=[]
    for entry in (PROC if proc_root is None else Path(proc_root)).iterdir():
        if not entry.name.isdecimal():continue
        try:
            if (entry/'comm').read_text().strip()!='vicon_client':continue
        except (FileNotFoundError,ProcessLookupError,PermissionError):continue
        try:
            fields=(entry/'environ').read_bytes().split(b'\0')
        except (FileNotFoundError,ProcessLookupError):continue
        except PermissionError as error:
            raise ValueError('无法确认Vicon接收器ROS域，保持原进程不动：PID='+entry.name) from error
        values=[v.partition(b'=')[2] for v in fields if v.startswith(b'ROS_DOMAIN_ID=')]
        value=values[0] if values else b'0'
        if len(values)>1 or not value or any(c<48 or c>57 for c in value):
            raise ValueError('Vicon接收器ROS域不明确，保持原进程不动：PID='+entry.name)
        result.append(dict(pid=int(entry.name),domain=int(value)))
    return sorted(result,key=lambda r:r['pid'])


def receiver_pids(domain=0,proc_root=None):
    return [v['pid'] for v in receiver_inventory(proc_root) if v['domain']==domain]


def verify_existing():
    sys.path.insert(0,str(PROVENANCE))
    from vicon_receiver_provenance import verify_current
    provenance=verify_current()
    inventory=receiver_inventory()
    matches=[r for r in inventory if r['domain']==0]
    if len(matches)!=1:raise ValueError('需要域0内唯一、已经运行的Vicon接收器；其他ROS域不参与小车判定')
    process=PROC/str(matches[0]['pid'])
    if str((process/'exe').resolve())!=provenance['installed']:
        raise ValueError('现有Vicon程序来源与已核验版本不符；停止只读预览')
    args=(process/'cmdline').read_bytes().decode().split('\0')
    expected=dict(hostname='10.10.10.2',namespace='vicon',world_frame='map',vicon_frame='vicon',
        map_xyz='[0.0,0.0,0.0]',map_rpy='[0.0,0.0,0.0]',map_rpy_in_degrees='false')
    # Do not accept a correct-looking old argument followed by an overriding
    # wrong frame/host. Same-domain conflicts still fail rather than starting
    # another receiver or silently reusing a differently configured source.
    if any([arg.partition(':=')[2] for arg in args if arg.partition(':=')[0]==key]!=[value]
            for key,value in expected.items()):
        raise ValueError('域0内Vicon坐标系、话题前缀或服务器与小车基线不同；不更改配置')
    if os.environ.get('ROS_DOMAIN_ID','0') not in ('0','124'):raise ValueError('只读入口要求ROS域0')
    return dict(pid=int(process.name),installed=provenance['installed'],
        installed_sha256=provenance['installed_sha256'],manifest=provenance['manifest'],
        manifest_sha256=provenance['manifest_sha256'],receiver_started=False,
        receiver_changed=False,actuator_outputs=False,receiver_selection='ROS_DOMAIN_0_ONLY',
        ignored_other_domains=[r for r in inventory if r['domain']!=0])


def source_ok(infos):
    # This is a graph identity check, not cryptographic DDS authentication.
    # Executable provenance is checked separately before subscribing.
    return len(infos)==1 and infos[0].node_name=='vicon_client' and infos[0].node_namespace=='/'
