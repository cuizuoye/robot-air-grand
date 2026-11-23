"""Offline provenance cases A-H; NEVER start a ROS context or SDK process."""
from copy import deepcopy
import hashlib
import json
import os
import struct
from pathlib import Path

import pytest

import vicon_receiver_provenance as p


def mock_elf_header():
    raw=bytearray(64); raw[:7]=b'\x7fELF\x02\x01\x01'
    struct.pack_into('<HHI',raw,16,3,62,1)
    return bytes(raw)


def mapped(path,perms='r-xp',*,suffix=''):
    info=Path(path).stat()
    return f'1000-2000 {perms} 00000000 {os.major(info.st_dev):02x}:{os.minor(info.st_dev):02x} {info.st_ino} {path}{suffix}\n'


@pytest.fixture
def pair(tmp_path):
    manifest=json.loads(p.MANIFEST.read_text())
    build=tmp_path/'build.elf'; installed=tmp_path/'installed.elf'
    build.write_bytes(Path(manifest['build']).read_bytes())
    installed.write_bytes(Path(manifest['installed']).read_bytes())
    return build,installed,deepcopy(manifest['receiver'])


def changed(path,offset,value=None):
    data=bytearray(path.read_bytes())
    data[offset]=data[offset]^1 if value is None else value
    path.write_bytes(data)


def test_A_exact_reviewed_files_can_be_identical(pair):
    build,installed,policy=pair
    installed.write_bytes(build.read_bytes())
    assert p.verify_installation(build,installed,policy)['change']=='identical_reviewed_ELF'


def test_B_only_reviewed_runpath_rewrite_passes(pair):
    result=p.verify_installation(*pair)
    assert result['change']=='exact_CMake_RUNPATH_rewrite_only'
    assert result['build_sha256']!=result['installed_sha256']
    # The approved persistent build path is longer than the old /tmp path.
    # Check actual changed offsets, not that historical byte-count accident.
    before,after=(path.read_bytes() for path in pair[:2])
    elf=p.Elf(before)
    start=elf.strings[4]+elf.run_offset
    changed_offsets={i for i,(a,b) in enumerate(zip(before,after)) if a!=b}
    assert len(before)==len(after) and changed_offsets
    assert changed_offsets<=set(range(start,start+len(pair[2]['build_runpath'].encode())+1))
    assert result['changed_bytes']==len(changed_offsets)
    assert p.Elf(after).runpath==pair[2]['install_runpath']
    assert result['full_ELF_comparison_except_exact_RUNPATH']


def test_C_modified_executable_code_refused(pair):
    build,installed,policy=pair
    elf=p.Elf(installed.read_bytes())
    changed(installed,elf.sections['.text'][4]+32)
    with pytest.raises(p.ProvenanceError,match='unexplained_ELF_difference'):
        p.verify_installation(build,installed,policy)


def test_D_changed_needed_library_refused(pair):
    build,installed,policy=pair
    data=bytearray(installed.read_bytes()); start=data.index(b'libViconDataStreamSDK_CPP.so\0')
    data[start+3]=ord('X'); installed.write_bytes(data)
    with pytest.raises(p.ProvenanceError,match='DT_NEEDED_changed'):
        p.verify_installation(build,installed,policy)


def test_E_unapproved_runpath_refused(pair):
    build,installed,policy=pair
    data=bytearray(installed.read_bytes()); elf=p.Elf(data)
    off=elf.strings[4]+elf.run_offset
    replacement=b'/tmp/evil_lib\0'
    data[off:off+len(replacement)]=replacement
    installed.write_bytes(data)
    with pytest.raises(p.ProvenanceError,match='unapproved_install_RUNPATH'):
        p.verify_installation(build,installed,policy)


def test_F_wrong_build_even_if_both_copies_match_refused(pair):
    build,installed,policy=pair
    elf=p.Elf(build.read_bytes()); changed(build,elf.sections['.text'][4]+32)
    installed.write_bytes(build.read_bytes())
    with pytest.raises(p.ProvenanceError,match='unreviewed_build_version'):
        p.verify_installation(build,installed,policy)


@pytest.mark.parametrize('missing',[0,1])
def test_G_missing_artifact_refused(pair,missing):
    pair[missing].unlink()
    with pytest.raises(p.ProvenanceError,match='missing_or_unreadable'):
        p.verify_installation(*pair)


def test_H_failed_gate_never_imports_ROS_or_starts_SDK(tmp_path,monkeypatch):
    import builtins
    import subprocess
    import vicon_frame_sample_capture as entry
    (tmp_path/'preparation_runs').mkdir()
    monkeypatch.setattr(entry,'ROOT',tmp_path)
    def refuse(): raise p.ProvenanceError('offline_intentional_bad_provenance')
    monkeypatch.setattr(entry,'verify_current',refuse)
    monkeypatch.setattr(subprocess,'Popen',lambda *a,**k:pytest.fail('SDK process launched after failed gate'))
    original=builtins.__import__
    def guarded(name,*a,**k):
        if name.startswith(('rclpy','vicon_receiver.msg')): pytest.fail('ROS import before successful gate')
        return original(name,*a,**k)
    monkeypatch.setattr(builtins,'__import__',guarded)
    with pytest.raises(p.ProvenanceError,match='intentional_bad'):
        entry.run('OFFLINE_MOCK_not_hardware_authorization')
    reports=list((tmp_path/'preparation_runs').glob('*/report.json'))
    assert len(reports)==1
    doc=json.loads(reports[0].read_text())
    assert not doc['receiver_started'] and not doc['ROS_initialized']


def test_other_dynstr_symbol_is_NOT_ignored(pair):
    build,installed,policy=pair
    elf=p.Elf(installed.read_bytes())
    changed(installed,elf.strings[4]+4)
    with pytest.raises(p.ProvenanceError,match='unexplained_ELF_difference'):
        p.verify_installation(*pair)


def test_runpath_padding_is_NOT_ignored(pair):
    build,installed,policy=pair
    elf=p.Elf(installed.read_bytes())
    off=elf.strings[4]+elf.run_offset+len(elf.runpath.encode())+1
    changed(installed,off,1)
    with pytest.raises(p.ProvenanceError,match='unexplained_ELF_difference'):
        p.verify_installation(*pair)


@pytest.mark.parametrize('section',['.rodata','.dynamic','.note.gnu.build-id'])
def test_other_critical_sections_not_ignored(pair,section):
    build,installed,policy=pair
    elf=p.Elf(installed.read_bytes())
    off=elf.sections[section][4]+(32 if section=='.dynamic' else 0)
    changed(installed,off)
    with pytest.raises(p.ProvenanceError): p.verify_installation(*pair)


def test_unknown_trailing_bytes_refused(pair):
    build,installed,policy=pair
    installed.write_bytes(installed.read_bytes()+b'unreviewed')
    with pytest.raises(p.ProvenanceError,match='unexplained_ELF_difference'):
        p.verify_installation(*pair)


def test_loader_output_records_canonical_actual_paths(tmp_path):
    lib=tmp_path/'lib_test.so'; lib.write_bytes(b'not executed')
    text=f'linux-vdso.so.1 (0x123)\nlib_test.so => {lib} (0x345)\n'
    assert p.parse_loader_list(text)=={'lib_test.so':str(lib)}


@pytest.mark.parametrize('text',['liboops.so => not found','', '/relative.so invalid-format'])
def test_unresolved_loader_output_refused(text):
    with pytest.raises(p.ProvenanceError): p.parse_loader_list(text)


def test_child_environment_cannot_inherit_preload_or_custom_search():
    policy=json.loads(p.MANIFEST.read_text())
    base=dict(LD_PRELOAD='/tmp/injected.so',LD_AUDIT='/tmp/audit.so',LD_LIBRARY_PATH='.:/tmp',
        LD_TRACE_LOADED_OBJECTS='1',RMW_IMPLEMENTATION='evil',ROS_DOMAIN_ID='0',
        FASTRTPS_DEFAULT_PROFILES_FILE='/tmp/plugins.xml',GLIBC_TUNABLES='x')
    safe=p.child_environment(base,policy)
    assert safe['ROS_DOMAIN_ID']=='83' and base['ROS_DOMAIN_ID']=='0'
    assert safe['RMW_IMPLEMENTATION']=='rmw_fastrtps_cpp'
    assert safe['LD_LIBRARY_PATH']==':'.join(policy['library_search_paths'])
    assert not {'LD_PRELOAD','LD_AUDIT','LD_TRACE_LOADED_OBJECTS','GLIBC_TUNABLES','FASTRTPS_DEFAULT_PROFILES_FILE'}&set(safe)


def test_library_same_name_changed_content_refused(tmp_path):
    lib=tmp_path/'lib.so'; lib.write_bytes(b'\x7fELFgood')
    expected={str(lib):{'sha256':p.digest(lib.read_bytes())}}
    assert p.verify_library_files(expected)
    lib.write_bytes(b'\x7fELFevil')
    with pytest.raises(p.ProvenanceError,match='library_content_unreviewed'):
        p.verify_library_files(expected)


def test_library_symlink_redirect_refused(tmp_path):
    lib=tmp_path/'lib.so'; other=tmp_path/'other.so'; lib.write_bytes(b'\x7fELFgood')
    expected={str(lib):{'sha256':p.digest(lib.read_bytes())}}
    other.write_bytes(lib.read_bytes()); lib.unlink(); lib.symlink_to(other)
    with pytest.raises(p.ProvenanceError,match='canonical_path_changed'):
        p.verify_library_files(expected)


def test_library_resolution_not_automatically_approved():
    policy=json.loads(p.MANIFEST.read_text())
    def bad(target,env):
        expected=deepcopy(policy['loader_resolutions'][target])
        expected['libnew_unapproved.so']='/tmp/libnew_unapproved.so'
        return expected
    with pytest.raises(p.ProvenanceError,match='runtime_dependency_resolution_changed'):
        p.verify_current(resolver=bad)


def test_actual_local_artifacts_with_mock_loader_need_no_ROS_or_SDK():
    policy=json.loads(p.MANIFEST.read_text())
    result=p.verify_current(resolver=lambda target,env:policy['loader_resolutions'][target])
    assert result['verified'] and result['library_count']>=100
    assert not result['receiver_started'] and not result['flight_authorized']


def test_unknown_runtime_mapping_refused(tmp_path):
    library=tmp_path/'unknown.so'; library.write_bytes(mock_elf_header())
    binary=tmp_path/'exe'; binary.write_bytes(mock_elf_header())
    provenance=dict(library_file_identities={},installed=str(binary),receiver_file_identity=p.file_identity(binary))
    line=mapped(library)
    with pytest.raises(p.ProvenanceError,match='unreviewed_mapped_library'):
        p.verify_runtime_maps(line,provenance)


def test_reviewed_mapping_file_replacement_refused(tmp_path):
    binary=tmp_path/'exe'; binary.write_bytes(mock_elf_header())
    provenance=dict(library_file_identities={},installed=str(binary),receiver_file_identity=p.file_identity(binary))
    line=mapped(binary)
    assert p.verify_runtime_maps(line,provenance)==[str(binary)]
    binary.write_bytes(mock_elf_header()+b'changed')
    with pytest.raises(p.ProvenanceError,match='mapped_file_changed'):
        p.verify_runtime_maps(line,provenance)
    with pytest.raises(p.ProvenanceError,match='changed_before_spawn'):
        p.recheck_before_spawn(provenance)


@pytest.mark.parametrize('bad',[b'',b'not an ELF',b'\x7fELF\x02\x01\x01'+b'\0'*60])
def test_malformed_ELF_refused_without_execution(bad):
    with pytest.raises(p.ProvenanceError): p.Elf(bad)


def test_deleted_runtime_mapping_refused(tmp_path):
    pfile=tmp_path/'known.so'; pfile.write_bytes(mock_elf_header())
    provenance=dict(library_file_identities={},installed=str(pfile),receiver_file_identity=p.file_identity(pfile))
    with pytest.raises(p.ProvenanceError,match='deleted_runtime_ELF'):
        p.verify_runtime_maps(mapped(pfile,suffix=' (deleted)'),provenance)


@pytest.mark.parametrize('paths',[['.'],[''],['/safe:/tmp'],['relative/path']])
def test_unsafe_library_search_paths_refused(paths):
    policy=json.loads(p.MANIFEST.read_text()); policy['library_search_paths']=paths
    with pytest.raises(p.ProvenanceError,match='unsafe_library_search'):
        p.child_environment({},policy)


def test_interpreter_symlink_redirect_before_spawn_refused(tmp_path):
    original=tmp_path/'loader'; original.write_bytes(b'\x7fELF')
    other=tmp_path/'other'; other.write_bytes(b'\x7fELF')
    alias=tmp_path/'interpreter'; alias.symlink_to(other)
    provenance=dict(library_file_identities={},installed=str(original),receiver_file_identity=p.file_identity(original),
        interpreter_request=str(alias),interpreter_resolved=str(original))
    with pytest.raises(p.ProvenanceError,match='interpreter_changed_before_spawn'):
        p.recheck_before_spawn(provenance)
