"""Parse the exact launch argv with installed Jazzy librcl, without rcl_init.

No ROS context/node/DDS/SDK is created. ctypes layouts below match the installed
rcl/arguments.h, rcl/log_level.h and rcutils/allocator.h. This is an offline test
helper only, not another production launch gate.
"""
import ast
import ctypes as c
from pathlib import Path

import pytest

import vicon_frame_sample_capture as entry


ROS=Path('/opt/ros/jazzy')


class Allocator(c.Structure):
    _fields_=[(name,c.c_void_p) for name in ('allocate','deallocate','reallocate','zero_allocate','state')]


class Arguments(c.Structure):
    _fields_=[('impl',c.c_void_p)]


class LogLevels(c.Structure):
    _fields_=[('default_logger_level',c.c_int),('logger_settings',c.c_void_p),
              ('num_logger_settings',c.c_size_t),('capacity_logger_settings',c.c_size_t),
              ('allocator',Allocator)]


def parse_ros_arguments(command):
    """Only librcl's memory-only parser/remap functions; never rcl_init()."""
    rcutils=c.CDLL(str(ROS/'lib/librcutils.so'))
    rcl=c.CDLL(str(ROS/'lib/librcl.so'))
    def bind(lib,name,argtypes,restype=c.c_int):
        fn=getattr(lib,name); fn.argtypes=argtypes; fn.restype=restype
        return fn
    allocator=bind(rcutils,'rcutils_get_default_allocator',[],Allocator)()
    args=bind(rcl,'rcl_get_zero_initialized_arguments',[],Arguments)()
    parse=bind(rcl,'rcl_parse_arguments',[c.c_int,c.POINTER(c.c_char_p),Allocator,c.POINTER(Arguments)])
    fini=bind(rcl,'rcl_arguments_fini',[c.POINTER(Arguments)])
    reset=bind(rcutils,'rcutils_reset_error',[],None)
    levels=bind(rcl,'rcl_get_zero_initialized_log_levels',[],LogLevels)()
    levels_ready=False
    encoded=(c.c_char_p*len(command))(*(v.encode() for v in command))
    result=dict(returncode=parse(len(command),encoded,allocator,c.byref(args)),context_created=False)
    try:
        if result['returncode']!=0:
            reset()
            return result
        result['unparsed_ros_count']=bind(rcl,'rcl_arguments_get_count_unparsed_ros',[c.POINTER(Arguments)])(c.byref(args))
        assert bind(rcl,'rcl_arguments_get_log_levels',[c.POINTER(Arguments),c.POINTER(LogLevels)])(c.byref(args),c.byref(levels))==0
        levels_ready=True
        result['default_log_level']=levels.default_logger_level
        release=c.CFUNCTYPE(None,c.c_void_p,c.c_void_p)(allocator.deallocate)
        node_remap=bind(rcl,'rcl_remap_node_name',
            [c.POINTER(Arguments),c.POINTER(Arguments),c.c_char_p,Allocator,c.POINTER(c.c_void_p)])
        topic_remap=bind(rcl,'rcl_remap_topic_name',
            [c.POINTER(Arguments),c.POINTER(Arguments),c.c_char_p,c.c_char_p,c.c_char_p,Allocator,c.POINTER(c.c_void_p)])
        def remap(fn,*prefix):
            output=c.c_void_p()
            assert fn(None,c.byref(args),*prefix,allocator,c.byref(output))==0
            try: return c.string_at(output).decode() if output.value else None
            finally:
                if output.value: release(output,allocator.state)
        node=remap(node_remap,b'vicon_client')
        result['remaps']=dict(node=node,
            tf=remap(topic_remap,b'/tf',node.encode(),b'/'),
            tf_static=remap(topic_remap,b'/tf_static',node.encode(),b'/'))
        return result
    finally:
        if levels_ready:
            assert bind(rcl,'rcl_log_levels_fini',[c.POINTER(LogLevels)])(c.byref(levels))==0
        if args.impl: assert fini(c.byref(args))==0


def launch_command():
    # Evaluate only the existing literal argv expression, never call run().
    tree=ast.parse(Path(entry.__file__).read_text())
    commands=[node.value for node in ast.walk(tree) if isinstance(node,ast.Assign)
              and any(isinstance(t,ast.Name) and t.id=='command' for t in node.targets)]
    assert len(commands)==1
    return eval(compile(ast.Expression(commands[0]),entry.__file__,'eval'),
                {'__builtins__':{},'str':str,'BINARY':entry.BINARY})


def test_exact_current_launch_arguments_parse_in_real_ros_without_initialization():
    result=parse_ros_arguments(launch_command())
    assert result['returncode']==0 and result['unparsed_ros_count']==0
    assert result['default_log_level']==30  # RCUTILS_LOG_SEVERITY_WARN
    assert result['remaps']==dict(node='b3_vicon_isolated_receiver',
        tf='/vicon_b3_check/tf',tf_static='/vicon_b3_check/tf_static')
    assert result['context_created'] is False


def test_old_invalid_log_remap_is_reproduced_by_real_parser():
    command=launch_command(); command[-2:]=['-r','__log_level:=warn']
    assert parse_ros_arguments(command)['returncode']!=0


def test_real_parser_rejects_unknown_log_severity():
    command=launch_command(); command[-1]='not_a_log_level'
    assert parse_ros_arguments(command)['returncode']!=0


def test_all_existing_receiver_parameters_and_remaps_are_retained():
    command=launch_command()
    assert command.count('--ros-args')==1
    assert command[-2:]==['--log-level','warn']
    params=[command[i+1] for i,arg in enumerate(command) if arg=='-p']
    assert params==['hostname:=10.10.10.2','namespace:=vicon_b3_check',
        'capture_frame_samples:=true','capture_subject:=MircoAir','capture_segment:=MircoAir',
        'stream_mode:=ServerPush','buffer_size:=200','world_frame:=map','vicon_frame:=vicon_b3_check']
    remaps=[command[i+1] for i,arg in enumerate(command) if arg=='-r']
    assert remaps==['__node:=b3_vicon_isolated_receiver','/tf:=/vicon_b3_check/tf',
                    '/tf_static:=/vicon_b3_check/tf_static']
