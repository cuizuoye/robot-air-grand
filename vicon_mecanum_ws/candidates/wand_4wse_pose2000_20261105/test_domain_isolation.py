"""Fake/proc only: domain83 coexistence, exact domain0 identity, ownership.

No receiver, drone/car socket, ROS context or network setting is touched.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock,patch

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'host'));import bootstrap
import receiver_manager as rm
from mecanum_wand_4wse_control import rotation_receiver_guard as guard

PARAMS=dict(rm.PARAMS)
PROOF=dict(installed='/approved/vicon_client',installed_sha256='offline',manifest='offline',manifest_sha256='offline')

class Child:
    pid=200
    def __init__(self):self.code=None;self.actions=[]
    def poll(self):return self.code
    def terminate(self):self.actions.append('terminate')
    def kill(self):self.actions.append('kill')
    def wait(self,timeout):self.actions.append('wait');self.code=0

class Isolation(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.proc=self.root/'proc';self.proc.mkdir()
        self.module=types.ModuleType('vicon_receiver_provenance');self.module.verify_current=Mock(return_value=dict(PROOF))
        self.patches=[patch.object(guard,'PROC',self.proc),patch.dict(sys.modules,{'vicon_receiver_provenance':self.module}),
            patch.dict(os.environ,{'ROS_DOMAIN_ID':'0'})]
        for p in self.patches:p.start()
    def tearDown(self):
        for p in reversed(self.patches):p.stop()
        self.tmp.cleanup()
    def process(self,pid,domain=0,**kw):
        p=self.proc/str(pid);p.mkdir();(p/'comm').write_text('vicon_client\n')
        env=[] if domain is None else ['ROS_DOMAIN_ID='+str(domain)]
        (p/'environ').write_bytes(('\0'.join(env)+'\0').encode())
        (p/'exe').symlink_to(kw.get('exe',PROOF['installed']))
        params=dict(PARAMS);params.update(kw.get('params',{}));args=[PROOF['installed'],'--ros-args']
        for k,v in params.items():args+=['-p',k+':='+v]
        args+=kw.get('extra',[]);(p/'cmdline').write_bytes(('\0'.join(args)+'\0').encode());return p
    def foreign(self):return self.process(83,83,params=dict(world_frame='vicon_b3_check',vicon_frame='vicon_b3_check'))
    def snapshot(self,p):
        return {f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in p.iterdir() if f.is_file() and not f.is_symlink()}
    def manager(self,**kw):
        return rm.PassiveReceiver(self.root,{},discover=rm.receiver_pids,verify=guard.verify_existing,
            popen=Mock(side_effect=AssertionError('real spawn forbidden')),**kw)
    def test_only83_is_not_adopted_as_car_receiver(self):
        self.foreign();self.assertEqual(rm.receiver_pids(),[])
        with self.assertRaisesRegex(ValueError,'域0内唯一'):guard.verify_existing()
    def test_domain0_and83_coexist(self):
        self.process(200);foreign=self.foreign();before=self.snapshot(foreign)
        value=guard.verify_existing();self.assertEqual(value['pid'],200)
        self.assertEqual(value['ignored_other_domains'],[dict(pid=83,domain=83)])
        self.assertEqual(before,self.snapshot(foreign))
    def test_existing_domain0_reused_without_spawn_or_foreign_stop(self):
        self.process(200);foreign=self.foreign();before=self.snapshot(foreign);r=self.manager()
        with patch.object(r,'spawn',side_effect=AssertionError('must reuse')):
            result=r.start();r.close()
        self.assertFalse(result['owned']);self.assertEqual(before,self.snapshot(foreign))
    def test_foreign_only_starts_own_domain0_and_closes_only_own_child(self):
        foreign=self.foreign();before=self.snapshot(foreign);r=self.manager();child=Child()
        def spawn():self.process(200);r.child=child
        with patch.object(r,'spawn',side_effect=spawn) as called:
            result=r.start();r.close();called.assert_called_once()
        self.assertTrue(result['owned']);self.assertEqual(result['pid'],200)
        self.assertEqual(child.actions,['terminate','wait']);self.assertEqual(before,self.snapshot(foreign))
    def test_two_domain0_receivers_still_blocked(self):
        self.process(200);self.process(201);self.foreign();r=self.manager()
        with patch.object(r,'spawn') as spawn:
            with self.assertRaisesRegex(ValueError,'域0内多个'):r.start()
            spawn.assert_not_called()
    def test_wrong_domain0_frame_not_hidden_by_correct_foreign_receiver(self):
        self.process(200,params=dict(vicon_frame='vicon_b3_check'));self.foreign()
        with self.assertRaisesRegex(ValueError,'坐标系'):guard.verify_existing()
    def test_wrong_domain0_topic_prefix_blocked(self):
        self.process(200,params=dict(namespace='vicon_b3_check'));self.foreign()
        with self.assertRaisesRegex(ValueError,'话题前缀'):guard.verify_existing()
    def test_wrong_domain0_executable_blocked(self):
        self.process(200,exe='/other/vicon_client');self.foreign()
        with self.assertRaisesRegex(ValueError,'来源'):guard.verify_existing()
    def test_duplicate_overriding_coordinate_argument_blocked(self):
        self.process(200,extra=['-p','world_frame:=vicon_b3_check'])
        with self.assertRaisesRegex(ValueError,'坐标系'):guard.verify_existing()
    def test_missing_domain_environment_defaults_to0(self):
        self.process(200,None);self.foreign();self.assertEqual(rm.receiver_pids(),[200]);self.assertEqual(guard.verify_existing()['pid'],200)
    def test_ambiguous_domain_environment_not_silently_ignored(self):
        p=self.process(200);(p/'environ').write_bytes(b'ROS_DOMAIN_ID=0\0ROS_DOMAIN_ID=83\0')
        with self.assertRaisesRegex(ValueError,'不明确'):rm.receiver_pids()
    def test_invalid_domain_environment_not_silently_ignored(self):
        self.process(200,'garbage')
        with self.assertRaisesRegex(ValueError,'不明确'):rm.receiver_pids()
    def test_unreadable_receiver_domain_not_guessed_foreign(self):
        self.process(200);read=Path.read_bytes
        def denied(p):
            if p.name=='environ':raise PermissionError('offline denied')
            return read(p)
        with patch.object(Path,'read_bytes',denied):
            with self.assertRaisesRegex(ValueError,'无法确认'):rm.receiver_pids()
    def test_nonreceiver_processes_ignored(self):
        p=self.proc/'300';p.mkdir();(p/'comm').write_text('other\n');self.foreign()
        self.assertEqual(rm.receiver_pids(),[])
    def test_source_graph_exclusivity_not_relaxed(self):
        good=types.SimpleNamespace(node_name='vicon_client',node_namespace='/')
        self.assertTrue(guard.source_ok([good]));self.assertFalse(guard.source_ok([good,good]))
        self.assertFalse(guard.source_ok([types.SimpleNamespace(node_name='other',node_namespace='/')]))
    def test_caller_domain83_cannot_be_used_as_car_source(self):
        self.process(200);self.foreign()
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'83'}):
            with self.assertRaisesRegex(ValueError,'ROS域0'):guard.verify_existing()

if __name__=='__main__':unittest.main(verbosity=2)
