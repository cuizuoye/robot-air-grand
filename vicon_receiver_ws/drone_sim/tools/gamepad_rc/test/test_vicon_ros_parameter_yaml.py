"""ROS parameter compatibility from inert YAML only; no ROS/SDK/devices."""
import hashlib
import json
from pathlib import Path

import pytest
import yaml

import vicon_frame_sample_capture as entry


FIXTURES=Path(__file__).parent/'fixtures'


def parse_file(tmp_path,text):
    path=tmp_path/'offline_parameters.yaml'
    path.write_text(text)
    before=path.read_bytes()
    result=entry.shared_receiver_parameters(['vicon_client','--ros-args','--params-file',str(path)])
    assert path.read_bytes()==before
    return result


@pytest.mark.parametrize('tag,sequence_type',[('',list),('!!python/tuple ',tuple)])
def test_plain_and_tuple_vectors_keep_exact_types_order_values_and_units(tmp_path,tag,sequence_type):
    result=parse_file(tmp_path,'/**:\n  ros__parameters:\n'
        f'    map_xyz: {tag}[1.25, -2.5, 3.75]\n'
        f'    map_rpy: {tag}[0.125, -0.25, 0.5]\n'
        '    map_rpy_in_degrees: true\n    hostname: 10.10.10.2\n    namespace: vicon\n')
    assert type(result['map_xyz']) is sequence_type and type(result['map_rpy']) is sequence_type
    assert tuple(result['map_xyz'])==(1.25,-2.5,3.75)
    assert tuple(result['map_rpy'])==(0.125,-0.25,0.5)  # not converted to radians
    assert all(type(x) is float for k in ('map_xyz','map_rpy') for x in result[k])
    assert result['map_rpy_in_degrees'] is True
    assert result['hostname']=='10.10.10.2' and result['namespace']=='vicon'


def test_real_launch_configuration_copy_is_byte_exact_and_all_parameters_preserved():
    meta=json.loads((FIXTURES/'vicon_shared_process_20261021.json').read_text())
    source=FIXTURES/meta['config_fixture']
    raw=source.read_bytes()
    assert hashlib.sha256(raw).hexdigest()==meta['config_sha256']
    result=entry.shared_receiver_parameters([meta['argv'][0],'--ros-args','--params-file',str(source)])
    assert result==dict(buffer_size=200,enable_debug_data=False,enable_device_data=False,
        enable_marker_ray_data=False,hostname='10.10.10.2',map_rpy=(0.0,0.0,0.0),
        map_rpy_in_degrees=False,map_xyz=(0.0,0.0,0.0),namespace='vicon',
        publish_latency=True,stream_mode='ServerPush',vicon_frame='vicon',world_frame='map')
    assert type(result['buffer_size']) is int
    for key in ('map_xyz','map_rpy'):
        assert type(result[key]) is tuple and len(result[key])==3
        assert all(type(v) is float for v in result[key])
    assert source.read_bytes()==raw


@pytest.mark.parametrize('tag',[
    '!unknown [0.0, 0.0, 0.0]',
    '!!python/list [0.0, 0.0, 0.0]',
    '!!python/object/apply:os.system ["SHOULD_NEVER_EXECUTE"]',
    '!!python/object/new:tuple []',
    '!!python/name:os.system ""',
    '!!python/module:os ""',
    '!!python/tuple [!!python/object/apply:os.system ["NEVER"]]',
])
def test_unknown_and_python_object_tags_are_rejected_without_execution(tmp_path,monkeypatch,tag):
    monkeypatch.setattr(entry.os,'system',lambda *args:pytest.fail('unsafe Python object constructor'))
    monkeypatch.setattr(entry.subprocess,'Popen',lambda *a,**k:pytest.fail('process from unsafe YAML'))
    with pytest.raises(yaml.constructor.ConstructorError,match='could not determine a constructor'):
        parse_file(tmp_path,'/**:\n  ros__parameters:\n    extra: '+tag+'\n')


@pytest.mark.parametrize('value',['!!python/tuple {a: 0.0}','!!python/tuple scalar'])
def test_tuple_tag_requires_sequence_node(tmp_path,value):
    with pytest.raises(yaml.constructor.ConstructorError,match='tuple tag requires a sequence'):
        parse_file(tmp_path,'/**:\n  ros__parameters:\n    map_xyz: '+value+'\n')


@pytest.mark.parametrize('key,value,reason',[
    ('map_xyz','!!python/tuple [0.0, 1.0]','expected_three_finite_doubles'),
    ('map_rpy','[0.0, 0.0, 0.0, 0.0]','expected_three_finite_doubles'),
    ('map_xyz','!!python/tuple [0, 0, 0]','expected_three_finite_doubles'),
    ('map_rpy','[0.0, true, 0.0]','expected_three_finite_doubles'),
    ('map_xyz','[.nan, 0.0, 0.0]','expected_three_finite_doubles'),
    ('map_rpy','[0.0, .inf, 0.0]','expected_three_finite_doubles'),
    ('map_xyz','{x: 0.0, y: 0.0, z: 0.0}','expected_three_finite_doubles'),
    ('map_rpy_in_degrees','"false"','expected_bool'),
    ('hostname','123','expected_string'),
    ('namespace','[vicon]','expected_string'),
])
def test_invalid_parameter_types_and_lengths_refused_without_coercion(tmp_path,key,value,reason):
    with pytest.raises(ValueError,match='invalid_ROS_parameter:'+key+':'+reason):
        parse_file(tmp_path,'/**:\n  ros__parameters:\n    '+key+': '+value+'\n')


@pytest.mark.parametrize('document',[
    '[]','null','/**: []','/**:\n  ros__parameters: []','/**:\n  not_parameters: {}',
])
def test_abnormal_parameter_file_structure_refused(tmp_path,document):
    with pytest.raises(ValueError,match='invalid_ROS_parameter'):
        parse_file(tmp_path,document)


def test_inline_tuple_uses_same_safe_loader():
    result=entry.shared_receiver_parameters(['vicon_client','--ros-args',
        '-p','map_xyz:=!!python/tuple [1.0, 2.0, 3.0]'])
    assert result['map_xyz']==(1.0,2.0,3.0) and type(result['map_xyz']) is tuple


def test_standard_yaml_safe_loader_is_not_globally_modified():
    entry.shared_receiver_parameters(['vicon_client','-p','extra:=!!python/tuple [1.0]'])
    with pytest.raises(yaml.constructor.ConstructorError):
        yaml.safe_load('!!python/tuple [1.0]')
