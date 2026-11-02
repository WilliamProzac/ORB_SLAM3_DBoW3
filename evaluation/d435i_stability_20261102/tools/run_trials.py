#!/usr/bin/env python3
"""Prepare/bind/dry-run locally, or explicitly execute the frozen board schedule."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import time

from trial_common import (BINARY_NAMES, DEFAULT_ROOT, STAGES, TOOLS, bind_stage,
    load_plan, local, now, prepare, read_json, require, require_predecessors,
    resolve_run, review_failure, seal_run, sha, verify_freeze, verify_input_identity,
    verify_finished_round, write_json)


PREFLIGHT_CODE=r'''
from pathlib import Path
import hashlib,json,shutil,sys
p=Path(sys.argv[1]);shared=Path(sys.argv[2]);vocab=Path(sys.argv[3]);helpers=Path(sys.argv[4]);run=Path(sys.argv[5]);roles=[]
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
for q in Path('/proc').glob('[0-9]*'):
 try:
  name=(q/'comm').read_text().strip()
  if name in ('ros_rgbd','rgbd_v2_tum_rep','cc1plus','perf'):roles.append(dict(pid=int(q.name),comm=name))
 except OSError:pass
files={str(vocab.resolve()):sha(vocab)}
for n in ['rgbd_v2_sequence_monitor.py','localmap_resource_sample.py']:files[str((helpers/n).resolve())]=sha(helpers/n)
for f in shared.rglob('*'):
 if f.is_file():files[str(f.resolve())]=sha(f)
governor=Path('/sys/devices/system/cpu/cpufreq/policy0/scaling_governor')
print(json.dumps(dict(active=roles,free_bytes=shutil.disk_usage('/userdata').free,temperature_c=int(Path('/sys/class/thermal/thermal_zone0/temp').read_text())/1000,
 sha256={n:sha(p/n) for n in ['ros_rgbd','liborb_slam3_ros.so']},shared_files=files,governor=governor.read_text().strip() if governor.exists() else None,
 owned_run_exists=run.exists())))
'''

LOADED_CODE=r'''
from pathlib import Path
import hashlib,json,sys
r=Path(sys.argv[1]);p=Path(sys.argv[2]);pid=int((r/'slam.pid').read_text());q=Path('/proc')/str(pid)
maps=(q/'maps').read_text();cores=sorted({line.split()[-1] for line in maps.splitlines() if 'liborb_slam3_ros.so' in line})
assert cores==[str((p/'liborb_slam3_ros.so').resolve())],cores
env=dict(v.split('=',1) for v in (q/'environ').read_bytes().decode().split(chr(0)) if '=' in v)
print(json.dumps(dict(pid=pid,exe=str((q/'exe').resolve()),maps=maps,start_ticks=int((q/'stat').read_text().rsplit(')',1)[1].split()[19]),
 settings_sha256=hashlib.sha256((r/'d435i.yaml').read_bytes()).hexdigest(),actual_sha256={'ros_rgbd':hashlib.sha256((q/'exe').read_bytes()).hexdigest(),'liborb_slam3_ros.so':hashlib.sha256(Path(cores[0]).read_bytes()).hexdigest()},
 eager_bow=env.get('ORB_RGBD_EAGER_BOW'),stability_directory=env.get('ORB_RGBD_STABILITY_DIR'),stability_windows=env.get('ORB_RGBD_STABILITY_WINDOWS'))))
'''

POST_CODE=r'''
from pathlib import Path
import hashlib,json,shutil,sys
r=Path(sys.argv[1]);p=Path(sys.argv[2]);owned=[];active=[]
for role in ['slam','sampler']:
 f=r/(role+'.process.json')
 if not f.exists():continue
 rec=json.loads(f.read_text());q=Path('/proc')/str(rec['pid'])
 try:alive=int((q/'stat').read_text().rsplit(')',1)[1].split()[19])==int(rec['start_ticks'])
 except FileNotFoundError:alive=False
 owned.append(dict(rec,same_process_still_present=alive))
for q in Path('/proc').glob('[0-9]*'):
 try:
  comm=(q/'comm').read_text().strip()
  if comm in ['ros_rgbd','rgbd_v2_tum_rep']:active.append(dict(pid=int(q.name),comm=comm))
 except OSError:pass
print(json.dumps(dict(owned_processes=owned,all_owned_processes_exited=not any(x['same_process_still_present'] for x in owned),active=active,
 sha256={n:hashlib.sha256((p/n).read_bytes()).hexdigest() for n in ['ros_rgbd','liborb_slam3_ros.so']},atlas_files=[str(f) for f in r.rglob('*.osa')],free_bytes=shutil.disk_usage('/userdata').free)))
'''


def connection(plan):
    network=plan.get('network',{});host=network.get('board','sunrise@192.168.1.11')
    ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=3','-o','ServerAliveCountMax=2',host]
    return host,ssh


def ensure_master_port_free(host,port):
    # ROS's XML-RPC server permits address reuse. TIME_WAIT is not a live master.
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        sock.bind((host,port))


def node_command(plan,resolved,remote_dir):
    pkg=resolved['package'];prefix=pkg['remote_path'];network=plan.get('network',{});port=network.get('master_port',11431)
    host_ip=network.get('host_ip','192.168.1.3');board_ip=network.get('board_ip','192.168.1.11')
    shared=pkg['shared_lib_dir'];stability=remote_dir+'/stability' if resolved['observation'] else None
    env=['LD_LIBRARY_PATH='+prefix+':'+shared+':/opt/ros/noetic/lib:/usr/local/lib:/usr/hobot/lib:/usr/lib/aarch64-linux-gnu',
         'ORB_MEMORY_STATS=0','ROS_IP='+board_ip,'ROS_MASTER_URI=http://%s:%s'%(host_ip,port),
         'ORB_RGBD_EAGER_BOW='+('1' if resolved['bow_ready'] else '0')]
    if stability:env+=['ORB_RGBD_STABILITY_DIR='+stability,'ORB_RGBD_STABILITY_WINDOWS='+plan.get('diagnostic_windows','23-100,640-710,1320-1420,2180-2270,3380-3470,5000-5220')]
    node=['env','-u','ORB_RGBD_STABILITY_DIR','-u','ORB_RGBD_STABILITY_WINDOWS']+env+[prefix+'/ros_rgbd','_voc_file:='+pkg['vocabulary'],'_settings_file:='+remote_dir+'/d435i.yaml',
        '_rgb_topic:=/x5_8g_test/rgb','_depth_topic:=/x5_8g_test/depth','_rgb_queue:=8','_depth_queue:=8','_sync_queue:=10',
        '_enable_pangolin:=false','_timing_enabled:=false','_timing_output_dir:='+remote_dir+'/timing',
        '_field_benchmark_metrics:=false','_field_diagnostics_enabled:=false','_v2_enabled:=false',
        '_scale_enabled:=true','_scale_detail:=true','_scale_input_index:='+remote_dir+'/input_index.csv',
        '_scale_output_dir:='+remote_dir+'/timing','_scale_front_timeout_s:=60','__name:=slam','__ns:=/x5_8g_test']
    return ['python3',pkg['helpers_dir']+'/rgbd_v2_sequence_monitor.py','--mode','exec_owned','--run-dir',remote_dir,'--role','slam','--']+node


def run_one(root,plan,resolved):
    import shutil
    from summarize_trials import reduce_one
    rid=resolved['id'];pkg=resolved['package'];cfg=resolved['config'];run=root/'runs'/rid
    require(not run.exists(),'Existing unsealed/failed round retained; automatic retry prohibited: '+rid)
    run.mkdir(parents=True,exist_ok=False);write_json(run/'RESOLVED_RUN.json',resolved,exclusive=True)
    source_index=root/'inputs'/rid/'input_index.csv';settings=run/'d435i.yaml';shutil.copyfile(local(root,cfg['path']),settings)
    host,ssh=connection(plan);network=plan.get('network',{});host_ip=network.get('host_ip','192.168.1.3');port=network.get('master_port',11431)
    remote_root=plan.get('remote_root','/userdata/rgbd_stability_trials_20261102')
    require(remote_root.startswith('/userdata/') and '..' not in Path(remote_root).parts,'Unsafe remote experiment path')
    remote_dir=remote_root+'/runs/'+rid;helpers=pkg['helpers_dir'];processes={};logs=[];created=False
    env=dict(os.environ,ROS_IP=host_ip,ROS_MASTER_URI='http://%s:%s'%(host_ip,port),ROS_HOME=str(run/'ros_home'))
    result=dict(original_entry_for_result(plan),id=rid,package=pkg['remote_path'],source_commit=pkg['source_commit'],
        settings_sha256=cfg['sha256'],rate=1.,queues=[8,8,10],scale_detail=True,start_wall_s=time.time(),
        resolved_identity_sha256=resolved['resolved_identity_sha256'],stage=resolved['stage'],config_id=resolved['config_id'],package_id=resolved['package_id'],
        observation=resolved['observation'],bow_ready=resolved['bow_ready'],remote_dir=remote_dir,
        stability_directory=remote_dir+'/stability' if resolved['observation'] else None,complete=False)
    def save(phase):result['phase']=phase;write_json(run/'RESULT.json',result);print(rid,phase,flush=True)
    def remote(code,*args,timeout=30):
        return json.loads(subprocess.check_output(ssh+['python3 - '+shlex.join([str(a) for a in args])],input=code,text=True,timeout=timeout))
    def spawn(role,cmd,filename=None):
        f=(run/(filename or role+'.log')).open('xb');logs.append(f)
        p=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes[role]=p;return p
    def stop(role):
        p=processes.get(role)
        if p is None or p.poll() is not None:return
        for sig,wait in [(signal.SIGINT,8),(signal.SIGTERM,3),(signal.SIGKILL,3)]:
            if p.poll() is not None:break
            require(os.getpgid(p.pid)==p.pid,'Refusing to signal an unowned host process group')
            try:os.killpg(p.pid,sig)
            except ProcessLookupError:break
            try:p.wait(timeout=wait)
            except subprocess.TimeoutExpired:pass
    def owned_stop(role):
        command=['python3',helpers+'/rgbd_v2_sequence_monitor.py','--mode','stop_owned','--run-dir',remote_dir,'--role',role,'--signal','INT']
        return subprocess.run(ssh+[shlex.join(command)],capture_output=True,text=True,timeout=25)
    def wait_until(predicate,seconds,reason,alive=()):
        deadline=time.monotonic()+seconds
        while not predicate():
            require(all(processes[r].poll() is None for r in alive),'Child exited: '+reason)
            require(time.monotonic()<deadline,'Timed out: '+reason);time.sleep(.2)
    try:
        save('preflight');verify_freeze(root);verify_input_identity(root,plan)
        require(sha(settings)==cfg['sha256'],'Round YAML changed')
        ensure_master_port_free(host_ip,port)
        board=remote(PREFLIGHT_CODE,pkg['remote_path'],pkg['shared_lib_dir'],pkg['vocabulary'],helpers,remote_dir)
        write_json(run/'board_preflight.json',board)
        require(not board['active'],'Other board workload is active')
        require(not board['owned_run_exists'],'Board round directory already exists; no overwrite/retry')
        require(board['free_bytes']>=plan.get('min_board_free_bytes',1024**3),'Insufficient board output headroom')
        require(board['sha256']==pkg['binaries'],'Node/core package hashes mismatch')
        if plan.get('max_start_temperature_c') is not None:require(board['temperature_c']<=plan['max_start_temperature_c'],'Board above frozen start temperature limit')
        for name in ['rgbd_v2_sequence_monitor.py','localmap_resource_sample.py']:
            require(any(path.endswith('/'+name) and digest==sha(TOOLS/name) for path,digest in board['shared_files'].items()),'Read-only remote helper differs from frozen helper: '+name)
        shared_pin=root/'preflight/SHARED_RUNTIME_IDENTITY.json'
        shared=dict(files=board['shared_files'],governor=board.get('governor'))
        if shared_pin.exists():require(read_json(shared_pin)==shared,'Shared libraries/vocabulary/CPU policy changed between rounds')
        else:write_json(shared_pin,shared,exclusive=True)
        remote('from pathlib import Path\nimport json,sys\np=Path(sys.argv[1]);p.mkdir(parents=True,exist_ok=False);(p/"timing").mkdir();(p/"stability").mkdir();print(json.dumps({"created":True}))\n',remote_dir);created=True
        subprocess.run(['rsync','-a','--timeout=15',str(source_index),str(settings),host+':'+remote_dir+'/'],check=True,timeout=30)
        spawn('master',['roscore','-p',str(port)])
        wait_until(lambda:subprocess.run(['rosnode','list'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=3).returncode==0,30,'ROS master',('master',))
        subprocess.run(['rosparam','set','/use_sim_time','false'],env=env,check=True,timeout=5)
        cmd=node_command(plan,resolved,remote_dir);result['node_command']=cmd;save('node_start')
        child=spawn('node',ssh+['source /opt/ros/noetic/setup.bash && exec '+shlex.join(cmd)])
        def ready():
            info=subprocess.run(['rosnode','info','/x5_8g_test/slam'],env=env,capture_output=True,text=True,timeout=3)
            return info.returncode==0 and '/x5_8g_test/depth' in info.stdout
        wait_until(ready,120,'node subscribers',('node',))
        loaded=remote(LOADED_CODE,remote_dir,pkg['remote_path']);write_json(run/'loaded.json',loaded)
        require(loaded['exe']==pkg['remote_path']+'/ros_rgbd' and loaded['actual_sha256']==pkg['binaries'],'Actual loaded executable/core identity mismatch')
        require(loaded['settings_sha256']==cfg['sha256'],'Actual loaded YAML mismatch')
        require(loaded['eager_bow']==('1' if resolved['bow_ready'] else '0') and loaded['stability_directory']==result['stability_directory'],'Actual startup runtime switches mismatch')
        sampler=['python3',helpers+'/rgbd_v2_sequence_monitor.py','--mode','exec_owned','--run-dir',remote_dir,'--role','sampler','--','python3',helpers+'/localmap_resource_sample.py',str(loaded['pid'])]
        spawn('sampler',ssh+[shlex.join(sampler)],'memory.csv')
        spawn('observer',['/usr/bin/python3',str(TOOLS/'rgbd_v2_sequence_monitor.py'),'--mode','host','--index',str(source_index),'--output',str(run/'host_events.csv'),'--rgb-topic','/x5_8g_test/rgb','--depth-topic','/x5_8g_test/depth'])
        spawn('poses',['/usr/bin/python3',str(TOOLS/'pose_observer.py'),'--output',str(run/'poses'),'--index',str(source_index)])
        wait_until(lambda:(run/'host_events.ready').exists() and (run/'poses/ready').exists(),30,'input/pose observers',('observer','poses','node'))
        inp=plan['input'];player_cmd=['/opt/ros/noetic/lib/rosbag/play','--queue','100','--rate','1','--delay','2','--topics',inp['rgb_topic'],inp['depth_topic'],'--bags',str(local(root,inp['bag'])),inp['rgb_topic']+':=/x5_8g_test/rgb',inp['depth_topic']+':=/x5_8g_test/depth']
        result['player_command']=player_cmd;save('replay');player=spawn('player',player_cmd)
        budget=(inp['original_record_end_ns']-inp['original_record_start_ns'])/1e9+100;deadline=time.monotonic()+budget;heartbeat=time.monotonic()
        while player.poll() is None:
            require(child.poll() is None,'Node exited during playback');require(time.monotonic()<deadline,'Player timeout')
            if time.monotonic()-heartbeat>=30:print(rid,'replay heartbeat',flush=True);heartbeat=time.monotonic()
            time.sleep(.5)
        result['player_exit']=player.returncode;require(player.returncode==0,'Player failed')
        save('seal_front');seal=subprocess.run(['rosservice','call','/x5_8g_test/slam/scale_seal','{}'],env=env,capture_output=True,text=True,timeout=65)
        (run/'seal.log').write_text(seal.stdout+seal.stderr);result['seal_exit']=seal.returncode;require(seal.returncode==0,'Front seal failed')
        save('shutdown');result['node_exit']=child.wait(timeout=365)
        time.sleep(2);stop('observer');stop('poses')
        result['observer_exit']=processes['observer'].returncode;result['pose_observer_exit']=processes['poses'].returncode
        try:result['sampler_exit']=processes['sampler'].wait(timeout=5)
        except subprocess.TimeoutExpired:owned_stop('sampler');stop('sampler');result['sampler_exit']=processes['sampler'].returncode
        require(all(result[k]==0 for k in ['node_exit','observer_exit','pose_observer_exit','sampler_exit']),'Child/observer/sampler exit failure')
        result['complete']=True
    except BaseException as error:
        result['error']=repr(error);print(rid,'ERROR',repr(error),flush=True)
    finally:
        save('cleanup_and_collect')
        try:stop('player')
        except Exception as error:result['player_cleanup_error']=repr(error)
        if created:
            for role in ['slam','sampler']:
                try:
                    outcome=owned_stop(role);(run/(role+'_cleanup.log')).write_text(outcome.stdout+outcome.stderr);result[role+'_cleanup_exit']=outcome.returncode
                except Exception as error:result[role+'_cleanup_error']=repr(error)
        for role in ['node','sampler','observer','poses','master']:
            try:stop(role)
            except Exception as error:result[role+'_host_cleanup_error']=repr(error);result['complete']=False
        for f in logs:f.close()
        if created:
            try:
                hashes=remote('from pathlib import Path\nimport hashlib,json,sys\np=Path(sys.argv[1]);print(json.dumps({str(f.relative_to(p)):hashlib.sha256(f.read_bytes()).hexdigest() for f in p.rglob("*") if f.is_file()}))\n',remote_dir)
                write_json(run/'remote_sha256.json',hashes)
                subprocess.run(['rsync','-a','--timeout=15',host+':'+remote_dir+'/',str(run/'board')+'/'],check=True,timeout=120)
                require(all(sha(run/'board'/name)==digest for name,digest in hashes.items()),'Returned board artifact SHA mismatch');result['transfer_verified']=True
                post=remote(POST_CODE,remote_dir,pkg['remote_path']);write_json(run/'board_postflight.json',post)
                require(post['all_owned_processes_exited'] and not post['active'] and not post['atlas_files'] and post['sha256']==pkg['binaries'],'Board postflight process/package/Atlas mismatch')
            except Exception as error:result['collect_error']=repr(error);result['complete']=False
        try:
            ensure_master_port_free(host_ip,port)
            exited=all(p.poll() is not None for p in processes.values())
            write_json(run/'host_postflight.json',dict(master_port=port,bindable=True,owned_children_exited=exited))
            require(exited,'An owned host child did not exit')
        except Exception as error:result['host_postflight_error']=repr(error);result['complete']=False
        if any(result.get(role+'_cleanup_exit')!=0 for role in ['slam','sampler']):result['complete']=False
        result['elapsed_s']=time.time()-result['start_wall_s'];save('finished')
    if result['complete']:
        try:
            summary=reduce_one(root,plan,resolved);write_json(run/'SUMMARY.json',summary,exclusive=True)
            if summary['trial_valid']:seal_run(root,resolved);return True
            print(rid,'invalid measurements; retain round and stop',flush=True)
        except Exception as error:write_json(run/'REDUCTION_FAILURE.json',dict(error=repr(error)),exclusive=True)
    return False


def original_entry_for_result(plan):
    inp=plan['input']
    return {k:inp[k] for k in ('bag','bag_sha256','bag_bytes','bag_mtime_ns','expected_pairs','original_record_start_ns','original_record_end_ns')}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,default=DEFAULT_ROOT)
    parser.add_argument('--stage',choices=STAGES);parser.add_argument('--package',help='Assert frozen package ID or remote path; never override hashes')
    mode=parser.add_mutually_exclusive_group();mode.add_argument('--prepare',action='store_true');mode.add_argument('--execute',action='store_true');mode.add_argument('--bind-stage',choices=('bow','matrix'));mode.add_argument('--review-failure',metavar='RUN_ID')
    parser.add_argument('--decision',type=Path,help='Stage decision JSON for one-time binding');args=parser.parse_args();root=args.root.resolve()
    if args.prepare:print(json.dumps(prepare(root),ensure_ascii=False));return 0
    plan=load_plan(root)
    if args.bind_stage:
        require(args.decision is not None,'--decision is required');print(json.dumps(bind_stage(root,args.bind_stage,read_json(args.decision)),ensure_ascii=False));return 0
    if args.review_failure:
        require(args.decision is not None,'--decision is required');verify_freeze(root)
        run=next(r for r in plan['runs'] if r['id']==args.review_failure)
        print(json.dumps(review_failure(root,resolve_run(root,plan,run),read_json(args.decision)),ensure_ascii=False));return 0
    selected=[r for r in plan['runs'] if not args.stage or r['stage']==args.stage]
    if not args.execute:
        output=[]
        for run in selected:
            try:
                resolved=resolve_run(root,plan,run);output.append(dict(id=run['id'],resolved=resolved,remote_command=node_command(plan,resolved,plan.get('remote_root','/userdata/rgbd_stability_trials_20261102')+'/runs/'+run['id'])))
            except ValueError as error:output.append(dict(id=run['id'],pending_binding=str(error)))
        print(json.dumps(dict(dry_run=True,rounds=output),ensure_ascii=False,indent=2));return 0
    require(args.stage is not None,'Execution requires an explicit stage; no automatic conditional stage advance')
    verify_freeze(root);verify_input_identity(root,plan,full_bag=True)
    (root/'operations').mkdir(exist_ok=True)
    with (root/'operations/runner.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        for run in selected:
            resolved=resolve_run(root,plan,run)
            if args.package:require(args.package in (resolved['package_id'],resolved['package']['remote_path']),'Requested package differs from frozen run')
            run_dir=root/'runs'/run['id']
            if run_dir.exists():
                verify_finished_round(root,resolved);print(run['id'],'completed/reviewed, hashes verified; skipped',flush=True);continue
            require_predecessors(root,plan,run)
            if not run_one(root,plan,resolved):return 2
    return 0


if __name__=='__main__':raise SystemExit(main())
