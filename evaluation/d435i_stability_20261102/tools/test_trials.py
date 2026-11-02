#!/usr/bin/env python3
"""Targeted offline regressions. No SSH, ROS master, board or SLAM is started."""
import csv
import json
import os
from pathlib import Path
import socket
import tempfile
import unittest
from unittest.mock import patch

import run_trials
import summarize_trials
import trial_common as common


def fixture(root):
    bag=root/'bag';bag.write_bytes(b'original recording fixture')
    index=root/'index.csv';records=root/'records.csv'
    fields=['source_ordinal','source_stamp_ns','rgb_seq','depth_seq']
    with index.open('w') as a,records.open('w') as b:
        wa=csv.writer(a);wb=csv.writer(b);wa.writerow(fields);wb.writerow(fields+['rgb_record_ns','depth_record_ns'])
        for n in range(1,5879):
            row=[n,1793259039195566654+(n-1)*100000000,n+71,n+71]
            wa.writerow(row);wb.writerow(row+[row[1]+50000000,row[1]+49000000])
    configs={}
    for name,f,l in [('P0',700,5),('P1',1000,5),('P2',700,8),('P3',1000,8)]:
        p=root/(name+'.yaml');p.write_text('Camera.fps: 10\nORBextractor.nFeatures: %d\nORBextractor.nLevels: %d\nORBextractor.scaleFactor: 1.2\n'%(f,l));configs[name]=dict(path=str(p),sha256=common.sha(p))
    runs=[]
    for stage in ('observation','bow'):
        for n,on in enumerate([False,True,True,False]):runs.append(dict(id=stage+'_'+str(n),stage=stage,package_id='runtime',config_id='P0',observation=on if stage=='observation' else None,bow_ready=False if stage=='observation' else on))
    for n,cfg in enumerate(common.MATRIX):runs.append(dict(id='matrix_'+str(n),stage='matrix',package_id=None,config_id=cfg,observation=None,bow_ready=None))
    pkg=dict(remote_path='/userdata/new',source_commit='frozen-test',binaries={'ros_rgbd':'1'*64,'liborb_slam3_ros.so':'2'*64},shared_lib_dir='/userdata/readonly/lib',vocabulary='/userdata/readonly/Vocabulary/orbvoc.dbow3',helpers_dir='/userdata/readonly/tools')
    plan=dict(schema='rgbd_stability_trials_v1',input=dict(bag=str(bag),bag_sha256=common.sha(bag),bag_bytes=bag.stat().st_size,bag_mtime_ns=bag.stat().st_mtime_ns,
        index=str(index),index_sha256=common.sha(index),records=str(records),records_sha256=common.sha(records),expected_pairs=5878,original_record_start_ns=1,original_record_end_ns=587800000001,rgb_topic='/original/rgb',depth_topic='/original/depth'),configs=configs,packages={'runtime':pkg},runs=runs)
    common.write_json(root/'PLAN.json',plan)
    return plan


def dummy_success(root,plan,run):
    resolved=common.resolve_run(root,plan,run);path=root/'runs'/run['id'];path.mkdir(parents=True)
    common.write_json(path/'RESULT.json',dict(complete=True));common.write_json(path/'SUMMARY.json',dict(trial_valid=True));common.seal_run(root,resolved)
    return resolved


class TrialTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name);self.plan=fixture(self.root)

    def test_prepare_reuses_partial_inputs_without_touching_prior_identity(self):
        owner=self.root/'preflight/INPUT_IDENTITY.json';common.write_json(owner,dict(owner='original precheck'));before=owner.read_bytes()
        path=self.root/'inputs'/self.plan['runs'][0]['id'];path.mkdir(parents=True)
        (path/'input_index.csv').write_bytes((self.root/'index.csv').read_bytes())
        common.prepare(self.root);common.prepare(self.root)
        self.assertEqual(owner.read_bytes(),before);self.assertEqual(len(list((self.root/'inputs').glob('*/input_index.csv'))),16)
        self.assertEqual(common.verify_freeze(self.root)['input_identity']['pairs'],5878)

    def test_full_bag_hash_detects_same_metadata_replacement(self):
        path=self.root/'bag';st=path.stat();path.write_bytes(b'X'*st.st_size);os.utime(path,ns=(st.st_atime_ns,st.st_mtime_ns))
        with self.assertRaisesRegex(ValueError,'Full source bag SHA'):common.verify_input_identity(self.root,self.plan,True)

    def test_matrix_rejects_an_extra_parameter_change(self):
        path=self.root/'P1.yaml';path.write_text(path.read_text().replace('1.2','1.3'));self.plan['configs']['P1']['sha256']=common.sha(path);common.write_json(self.root/'PLAN.json',self.plan)
        with self.assertRaisesRegex(ValueError,'Unplanned YAML'):common.load_plan(self.root)

    def test_binding_requires_prior_stage_and_is_write_once(self):
        common.prepare(self.root);decision=dict(package_id='runtime',observation=False,decision_evidence={'rule':'fixture threshold review'})
        with self.assertRaises(FileNotFoundError):common.bind_stage(self.root,'bow',decision)
        for run in self.plan['runs'][:4]:dummy_success(self.root,self.plan,run)
        common.bind_stage(self.root,'bow',decision)
        with self.assertRaises(FileExistsError):common.bind_stage(self.root,'bow',decision)
        resolved=common.resolve_run(self.root,self.plan,self.plan['runs'][4]);self.assertFalse(resolved['observation']);self.assertFalse(resolved['bow_ready'])

    def test_sealed_round_cannot_be_changed_or_rerun(self):
        resolved=dummy_success(self.root,self.plan,self.plan['runs'][0]);common.verify_finished_round(self.root,resolved)
        common.write_json(self.root/'runs'/resolved['id']/'RESULT.json',dict(complete=False))
        with self.assertRaisesRegex(ValueError,'artifact changed'):common.verify_finished_round(self.root,resolved)

    def failure_files(self):
        resolved=common.resolve_run(self.root,self.plan,self.plan['runs'][0]);path=self.root/'runs'/resolved['id'];path.mkdir(parents=True)
        common.write_json(path/'RESULT.json',dict(phase='finished',complete=False,transfer_verified=True,slam_cleanup_exit=0,sampler_cleanup_exit=0,resolved_identity_sha256=resolved['resolved_identity_sha256'],stability_directory=None))
        common.write_json(path/'board_postflight.json',dict(sha256=resolved['package']['binaries'],all_owned_processes_exited=True,active=[],atlas_files=[]))
        common.write_json(path/'host_postflight.json',dict(bindable=True,owned_children_exited=True))
        common.write_json(path/'loaded.json',dict(actual_sha256=resolved['package']['binaries'],settings_sha256=resolved['config']['sha256'],eager_bow='0',stability_directory=None))
        (path/'board/timing').mkdir(parents=True);(path/'board/failure.txt').write_text('candidate overloaded')
        with (self.root/'index.csv').open() as f:original=next(csv.DictReader(f))
        fields=['source_ordinal','source_stamp_ns','rgb_seq','depth_seq']
        for name in ('scale_frames.csv','scale_sources.csv'):
            with (path/'board/timing'/name).open('w') as f:
                writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerow(original)
        with (path/'board/timing/scale_events.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['kind','source_ordinal','seq','stamp_ns','steady_ns']);writer.writerow(['received_rgb',original['source_ordinal'],original['rgb_seq'],original['source_stamp_ns'],1])
        with (path/'host_events.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['event','source_stamp_ns','rgb_seq','message_seq']);writer.writerow(['host_rgb',original['source_stamp_ns'],original['rgb_seq'],original['rgb_seq']])
        common.write_json(path/'host_events.status.json',dict(records=1,identity_duplicate_or_overflow_errors=0,callback_errors=0))
        common.write_json(path/'board/timing/completion.json',dict(identity_errors=0,callback_errors=0,cpu_clock_errors=0,frame_overflow=1))
        common.write_json(path/'remote_sha256.json',{str(p.relative_to(path/'board')):common.sha(p) for p in (path/'board').rglob('*') if p.is_file()})
        return resolved,path

    def test_reviewed_candidate_failure_stays_failed_and_can_be_skipped(self):
        resolved,path=self.failure_files();decision=dict(classification='candidate_overload',reason='Synthetic observed overload',evidence=['failure.txt'],continue_predeclared_schedule=True)
        common.review_failure(self.root,resolved,decision);common.verify_finished_round(self.root,resolved)
        self.assertFalse(common.read_json(path/'RESULT.json')['complete']);self.assertFalse(common.read_json(path/'REVIEWED_FAILURE.json')['trial_valid'])
        with self.assertRaisesRegex(ValueError,'failure cannot'):common.seal_run(self.root,resolved)

    def test_failure_review_cannot_waive_residual_process(self):
        resolved,path=self.failure_files();post=common.read_json(path/'board_postflight.json');post['active']=[{'pid':123}];common.write_json(path/'board_postflight.json',post)
        with self.assertRaisesRegex(ValueError,'cannot be waived'):common.review_failure(self.root,resolved,dict(classification='candidate_overload',reason='load',evidence=['log'],continue_predeclared_schedule=True))

    def test_failure_review_cannot_waive_observed_source_identity_mismatch(self):
        resolved,path=self.failure_files();frame=path/'board/timing/scale_frames.csv'
        frame.write_text(frame.read_text().replace(',72,72',',73,72'))
        hashes=common.read_json(path/'remote_sha256.json');hashes['timing/scale_frames.csv']=common.sha(frame);common.write_json(path/'remote_sha256.json',hashes)
        with self.assertRaisesRegex(ValueError,'source identity mismatch cannot be waived'):common.review_failure(self.root,resolved,dict(classification='candidate_overload',reason='load',evidence=['log'],continue_predeclared_schedule=True))

    def test_failure_review_cannot_waive_source_identity_error_counter(self):
        resolved,path=self.failure_files();completion=path/'board/timing/completion.json';value=common.read_json(completion);value['identity_errors']=1;common.write_json(completion,value)
        hashes=common.read_json(path/'remote_sha256.json');hashes['timing/completion.json']=common.sha(completion);common.write_json(path/'remote_sha256.json',hashes)
        with self.assertRaisesRegex(ValueError,'source identity/error counters cannot be waived'):common.review_failure(self.root,resolved,dict(classification='candidate_overload',reason='load',evidence=['log'],continue_predeclared_schedule=True))

    def test_runtime_switches_do_not_disable_base_identity_recorder(self):
        off=common.resolve_run(self.root,self.plan,self.plan['runs'][0]);on=common.resolve_run(self.root,self.plan,self.plan['runs'][1])
        a=run_trials.node_command(self.plan,off,'/userdata/new/run');b=run_trials.node_command(self.plan,on,'/userdata/new/run')
        self.assertIn('_scale_enabled:=true',a);self.assertIn('_scale_detail:=true',a)
        self.assertNotIn('ORB_RGBD_STABILITY_DIR=/userdata/new/run/stability',a)
        self.assertIn('ORB_RGBD_STABILITY_DIR=/userdata/new/run/stability',b);self.assertIn('ORB_RGBD_EAGER_BOW=0',a)
        self.assertIn('-u',a)

    def test_board_only_mapping_weights_and_merge_guard(self):
        index=[dict(source_ordinal=str(n),source_stamp_ns=str(n*1000000000)) for n in range(1,5)]
        frames=[]
        for n,state,fresh in [(1,2,1),(2,3,0),(4,2,1)]:
            frames.append(dict(source_ordinal=str(n),source_stamp_ns=str(n*1000000000),map_id='0',scale_snapshot_mono_ns='1',state_after=str(state),pose_valid=str(fresh),pose_fresh=str(fresh),global_kf='1',global_mp='700',tx=str(n),ty='0',tz='0'))
        result=summarize_trials.map_and_trajectory(frames,index,'MERGE DETECTED')
        self.assertEqual(result['time_weights']['nonfresh_s'],1);self.assertEqual(result['time_weights']['unknown_unprocessed_s'],1)
        self.assertIsNone(result['trajectory']['fresh_finite_translation_steps_same_epoch_consecutive_sources'])
        self.assertIsNone(result['new_map_creation_logs']['after_initial_atlas_creation'])

    def test_port_probe_rejects_a_real_listener(self):
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);server.bind(('127.0.0.1',0));server.listen(1)
            with self.assertRaises(OSError):run_trials.ensure_master_port_free('127.0.0.1',server.getsockname()[1])

    def test_port_probe_allows_time_wait_like_ros(self):
        server=socket.socket();server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);server.bind(('127.0.0.1',0));port=server.getsockname()[1];server.listen(1)
        client=socket.socket();client.connect(('127.0.0.1',port));accepted,_=server.accept()
        accepted.shutdown(socket.SHUT_WR);client.recv(1);client.close();accepted.close();server.close()
        run_trials.ensure_master_port_free('127.0.0.1',port)


if __name__=='__main__':unittest.main(verbosity=2)
