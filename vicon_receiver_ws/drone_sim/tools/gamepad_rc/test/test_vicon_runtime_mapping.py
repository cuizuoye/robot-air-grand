"""File-type mapping regression. No ROS initialization, SDK or device access."""
import json
import os
from pathlib import Path

import pytest

import vicon_receiver_provenance as p
from test_vicon_receiver_provenance import mock_elf_header, mapped


@pytest.fixture
def provenance():
    policy=json.loads(p.MANIFEST.read_text())
    return p.verify_current(resolver=lambda target,env:policy['loader_resolutions'][target])


def test_real_loader_cache_is_nonELF_data_not_a_library(provenance):
    cache=Path('/etc/ld.so.cache')
    assert cache.read_bytes().startswith(b'glibc-ld.so.cache')
    audit={}
    assert p.verify_runtime_maps(mapped(cache,'r--p'),provenance,details=audit)==[]
    assert audit['category_counts']=={'non_elf_file':1}
    assert not audit['mappings'][0]['library_approved']


@pytest.mark.parametrize('selector,category',[
    ('libc.so.6','elf_shared_object'),
    ('libViconDataStreamSDK_CPP.so','elf_shared_object'),
    ('librclcpp.so','elf_shared_object'),
    ('loader','elf_loader'),
    ('receiver','elf_executable'),
])
def test_actual_reviewed_system_SDK_ROS_loader_and_receiver(provenance,selector,category):
    if selector=='receiver': path=provenance['installed']
    elif selector=='loader': path=provenance['interpreter_resolved']
    else:
        resolved=provenance['loader_resolutions'][provenance['installed']]
        path=resolved[selector]
    audit={}
    assert p.verify_runtime_maps(mapped(path),provenance,details=audit)==[path]
    assert audit['mappings'][0]['category']==category
    assert audit['mappings'][0]['library_approved']


@pytest.mark.parametrize('name',['','[heap]','[stack]','[stack:42]','[anon:allocator]','[anon_shmem:pool]'])
def test_normal_anonymous_memory_allowed_without_library_approval(provenance,name):
    audit={}
    line=f'1000-2000 rw-p 00000000 00:00 0 {name}\n'
    assert p.verify_runtime_maps(line,provenance,details=audit)==[]
    assert audit['category_counts']=={'anonymous':1}


@pytest.mark.parametrize('name,perms',[
    ('[vdso]','r-xp'),('[vsyscall]','--xp'),('[vvar]','r--p'),('[vvar_vclock]','r--p')])
def test_known_kernel_mappings_not_mistaken_for_missing_files(provenance,name,perms):
    audit={}
    assert p.verify_runtime_maps(f'1000-2000 {perms} 0 00:00 0 {name}\n',provenance,details=audit)==[]
    assert audit['category_counts']=={'kernel_special':1}


@pytest.mark.parametrize('name',['configuration.data','ordinary.so.data','has spaces.cache'])
def test_regular_nonELF_data_is_allowed_independent_of_filename(tmp_path,provenance,name):
    data=tmp_path/name; data.write_bytes(b'plain mapped data\0')
    audit={}
    assert p.verify_runtime_maps(mapped(data,'r--p'),provenance,details=audit)==[]
    assert audit['mappings'][0]['category']=='non_elf_file'


@pytest.mark.parametrize('filename,perms',[('unknown.so','r-xp'),('disguised.data','r-xp'),('unknown','r--p')])
def test_unapproved_ELF_never_approved_by_suffix_or_readonly_segment(tmp_path,provenance,filename,perms):
    library=tmp_path/filename; library.write_bytes(mock_elf_header())
    with pytest.raises(p.ProvenanceError,match='unreviewed_mapped_library'):
        p.verify_runtime_maps(mapped(library,perms),provenance)


@pytest.mark.parametrize('destroy_header',[False,True])
def test_tampered_known_ELF_is_refused_even_if_replaced_with_nonELF(tmp_path,provenance,destroy_header):
    library=tmp_path/'reviewed'; library.write_bytes(mock_elf_header())
    provenance['library_file_identities'][str(library)]=p.verify_library_files({str(library):{'sha256':p.digest(library.read_bytes())}})[str(library)]
    line=mapped(library,'r--p')
    library.write_bytes(b'ordinary data now' if destroy_header else mock_elf_header()+b'tampered')
    with pytest.raises(p.ProvenanceError,match='mapped_file_changed'):
        p.verify_runtime_maps(line,provenance)


@pytest.mark.parametrize('name',['','[heap]','[anon:jit]','[unknown_kernel_name]'])
def test_unknown_executable_memory_refused(provenance,name):
    with pytest.raises(p.ProvenanceError,match='unapproved_'):
        p.verify_runtime_maps(f'1000-2000 rwxp 0 00:00 0 {name}\n',provenance)


def test_nonELF_file_with_execute_permission_refused(tmp_path,provenance):
    file=tmp_path/'text'; file.write_bytes(b'not ELF machine code')
    with pytest.raises(p.ProvenanceError,match='unapproved_non_elf_file_mapping'):
        p.verify_runtime_maps(mapped(file),provenance)


def test_missing_nonexecutable_file_recorded_unknown_not_approved_library(provenance):
    audit={}
    text='1000-2000 rw-s 0 00:01 42 /missing/unlinked_data (deleted)\n'
    assert p.verify_runtime_maps(text,provenance,details=audit)==[]
    assert audit['unknown_nonexecutable_count']==1
    assert not audit['mappings'][0]['library_approved']


def test_missing_executable_file_refused(provenance):
    with pytest.raises(p.ProvenanceError,match='unapproved_unrecognized_mapping'):
        p.verify_runtime_maps('1000-2000 r-xp 0 00:01 42 /missing/code\n',provenance)


def test_device_mapping_not_opened_as_file(tmp_path,provenance):
    fifo=tmp_path/'a_fifo'; os.mkfifo(fifo)
    audit={}
    # stat rejects a nonregular type BEFORE open/read; no FIFO blocking.
    assert p.verify_runtime_maps(mapped(fifo,'r--p'),provenance,details=audit)==[]
    assert audit['unknown_nonexecutable_count']==1


@pytest.mark.parametrize('text',['bad line','2000-1000 r-xp 0 00:00 0','1000-2000 bad! 0 00:00 0'])
def test_invalid_maps_syntax_refused(text,provenance):
    with pytest.raises(p.ProvenanceError): p.verify_runtime_maps(text,provenance)


def test_actual_mapping_fixture_has_honest_provenance_and_classifies(provenance):
    fixture=Path(__file__).parent/'fixtures/vicon_existing_process_maps_20261021.txt'
    meta=json.loads(fixture.with_suffix('.json').read_text())
    text=fixture.read_text()
    assert p.digest(text.encode())==meta['sha256']
    assert meta['failed_attempt_snapshot'] is False
    classified=[p.classify_mapping(p.parse_mapping(line),provenance) for line in text.splitlines()]
    categories={r['category'] for r in classified}
    assert {'anonymous','kernel_special','elf_loader','elf_shared_object'}<=categories
    # Historical shared-memory mappings may have disappeared since capture.
    # Do not pretend to know their old bytes or automatically approve a lib.
    shared_rows=[r for r in classified if r['path'].startswith('/dev/shm/')]
    assert shared_rows
    for row in shared_rows:
        assert not row['executable'] and not row['library_approved'] and row['allowed']
        if Path(row['path']).exists():
            assert row['category']=='non_elf_file'
        else:
            assert row['category']=='unrecognized'
            assert row['uncertainty']=='file_header_unavailable_nonexecutable'
    # This is a DIFFERENT existing process, not the approved candidate. Its
    # vendor/source paths must not be silently inserted into the real policy.
    assert any(not r['allowed'] for r in classified if r['category']=='elf_shared_object')


def test_mapping_inspected_once_per_snapshot_not_forever(tmp_path,provenance):
    data=tmp_path/'data'; data.write_bytes(b'ordinary')
    calls=[]
    def inspect(path):
        calls.append(path)
        return p.inspect_mapping_file(path)
    raw=mapped(data,'r--p')+mapped(data,'rw-p')
    p.verify_runtime_maps(raw,provenance,inspector=inspect)
    assert len(calls)==1
    p.verify_runtime_maps(raw,provenance,inspector=inspect)
    assert len(calls)==2
