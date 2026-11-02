#!/usr/bin/env python3
"""Frozen experiment identities and local-only state shared by runner/reducer."""
import csv
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil

TOOLS=Path(__file__).resolve().parent
DEFAULT_ROOT=TOOLS.parents[3]
STAGES=('observation','bow','matrix')
MATRIX=('P0','P1','P2','P3','P3','P2','P1','P0')
BINARY_NAMES=('ros_rgbd','liborb_slam3_ros.so')


def now():return datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='seconds')
def read_json(path):return json.loads(Path(path).read_text())
def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False)
def object_sha(value):return hashlib.sha256(canonical(value).encode()).hexdigest()


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(8*1024**2),b''):h.update(block)
    return h.hexdigest()


def write_json(path,value,exclusive=False):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    text=json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    if exclusive:
        with path.open('x') as f:f.write(text)
    else:
        tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(text);tmp.replace(path)


def local(root,value):
    p=Path(value)
    return p if p.is_absolute() else root/p


def require(condition,message):
    if not condition:raise ValueError(message)


def yaml_values(path):
    result={}
    for line in Path(path).read_text().splitlines():
        if line.startswith(('#','%')) or ':' not in line:continue
        key,value=line.split(':',1);result[key.strip()]=value.split('#',1)[0].strip()
    return result


def load_plan(root):
    plan=read_json(root/'PLAN.json')
    require(plan.get('schema')=='rgbd_stability_trials_v1','Unexpected PLAN schema')
    inp=plan['input'];require(inp['expected_pairs']==5878,'Expected all5878 original input pairs')
    require(plan.get('rate',1)==1 and plan.get('queues',[8,8,10])==[8,8,10],'Rate/queues must remain1x and8/8/10')
    runs=plan['runs'];require(len(runs)==16,'Frozen schedule must contain16 rounds')
    require(len({r['id'] for r in runs})==16,'Duplicate round IDs')
    require(all(re.fullmatch(r'[a-zA-Z0-9_-]+',r['id']) for r in runs),'Unsafe round ID')
    require([r['stage'] for r in runs]==['observation']*4+['bow']*4+['matrix']*8,'Stage order differs from predeclared16-round schedule')
    require([r['config_id'] for r in runs[:8]]==['P0']*8,'Both B controls use P0')
    require(tuple(r['config_id'] for r in runs[8:])==MATRIX,'Matrix order must be P0 P1 P2 P3 P3 P2 P1 P0')
    require([r['observation'] for r in runs[:4]]==[False,True,True,False] and all(r['bow_ready'] is False for r in runs[:4]),'Observation control must beoff/on/on/off with BoWoff')
    require([r['bow_ready'] for r in runs[4:8]]==[False,True,True,False],'BoW control must beoff/on/on/off')
    for stage,keys in [('observation',('package_id',)),('bow',('package_id','observation')),('matrix',('package_id','observation','bow_ready'))]:
        group=[r for r in runs if r['stage']==stage]
        for key in keys:require(len({r.get(key) for r in group})==1,'Confounded within-stage '+key)
    for run in runs:
        require(run['config_id'] in plan['configs'],'Unknown configuration')
        require(run.get('package_id') is None or run['package_id'] in plan['packages'],'Unknown package ID')
        require(all(run.get(k) is None or isinstance(run[k],bool) for k in ('observation','bow_ready')),'Runtime switches must be booleans or null pending stage binding')
    for pkg in plan['packages'].values():
        require(str(pkg['remote_path']).startswith('/userdata/'),'Package must have an explicit board/userdata path')
        require(set(pkg['binaries'])==set(BINARY_NAMES),'Package requires matched node/core expected hashes')
        require(all(re.fullmatch('[0-9a-f]{64}',v) for v in pkg['binaries'].values()),'Malformed package hashes')
        require(bool(pkg.get('source_commit')),'Package source commit required')
        for k in ('shared_lib_dir','vocabulary','helpers_dir'):require(str(pkg[k]).startswith('/'),'Package '+k+' requires absolute board path')
    expected={'P0':(700,5),'P1':(1000,5),'P2':(700,8),'P3':(1000,8)}
    base=yaml_values(local(root,plan['configs']['P0']['path']))
    require(float(base['Camera.fps'])==10,'Preserve10Hz configuration')
    for cfg,(features,levels) in expected.items():
        entry=plan['configs'][cfg];path=local(root,entry['path']);require(sha(path)==entry['sha256'],'YAML SHA mismatch: '+cfg)
        values=yaml_values(path);require(set(values)==set(base),'YAML keys differ: '+cfg)
        require(int(values['ORBextractor.nFeatures'])==features and int(values['ORBextractor.nLevels'])==levels,'Wrong matrix parameters: '+cfg)
        require(all(values[k]==base[k] for k in base if k not in ('ORBextractor.nFeatures','ORBextractor.nLevels')),'Unplanned YAML difference: '+cfg)
        require('System.LoadAtlasFromFile' not in values or values['System.LoadAtlasFromFile'] in ('','""'),'Each run must start without a loaded Atlas')
    require(sha(local(root,inp['index']))==inp['index_sha256'],'Index SHA mismatch')
    require(sha(local(root,inp['records']))==inp['records_sha256'],'Original record table SHA mismatch')
    return plan


def verify_input_identity(root,plan,full_bag=False):
    inp=plan['input'];bag=local(root,inp['bag']);st=bag.stat()
    require(st.st_size==inp['bag_bytes'] and st.st_mtime_ns==inp['bag_mtime_ns'],'Source bag metadata changed')
    if full_bag:require(sha(bag)==inp['bag_sha256'],'Full source bag SHA mismatch')
    with local(root,inp['index']).open() as f:index=list(csv.DictReader(f))
    with local(root,inp['records']).open() as f:records=list(csv.DictReader(f))
    require(len(index)==len(records)==5878,'Index/record table must contain all5878 pairs')
    for n,(a,b) in enumerate(zip(index,records),1):
        require(int(a['source_ordinal'])==n and all(a[k]==b[k] for k in ('source_ordinal','source_stamp_ns','rgb_seq','depth_seq')),'Original source identity mismatch')
    require(all(int(b['source_stamp_ns'])>int(a['source_stamp_ns']) for a,b in zip(index,index[1:])),'Source stamps not strictly ordered')
    return dict(full_bag_sha_verified=full_bag,bag_sha256=inp['bag_sha256'],pairs=5878,index_sha256=inp['index_sha256'],records_sha256=inp['records_sha256'])


def prepare(root):
    plan=load_plan(root);identity=verify_input_identity(root,plan,full_bag=True)
    freeze=root/'preflight/RUN_FREEZE.json'
    if freeze.exists():verify_freeze(root);return read_json(freeze)
    for run in plan['runs']:
        out=root/'inputs'/run['id'];out.mkdir(parents=True,exist_ok=True)
        for key,name,digest_key in [('index','input_index.csv','index_sha256'),('records','original_input_records.csv','records_sha256')]:
            destination=out/name
            if destination.exists():require(sha(destination)==plan['input'][digest_key],'Existing prepared input differs; never overwrite')
            else:shutil.copyfile(local(root,plan['input'][key]),destination)
    paths=[root/'PLAN.json']+list(TOOLS.glob('*.py'))
    paths += [local(root,c['path']) for c in plan['configs'].values()]
    paths += [p for p in (root/'inputs').rglob('*.csv')]
    if plan.get('protocol_file'):paths.append(local(root,plan['protocol_file']))
    for pkg in plan['packages'].values():
        for key in ('build_result','build_manifest'):
            if pkg.get(key):paths.append(local(root,pkg[key]))
    result=dict(schema='rgbd_stability_freeze_v1',created=now(),plan_sha256=sha(root/'PLAN.json'),input_identity=identity,files={str(p.resolve()):sha(p) for p in paths})
    identity_file=root/'preflight/RUNNER_INPUT_IDENTITY.json'
    if identity_file.exists():require(read_json(identity_file)==identity,'Prior runner input verification differs')
    else:write_json(identity_file,identity,exclusive=True)
    write_json(freeze,result,exclusive=True)
    return result


def verify_freeze(root):
    freeze=read_json(root/'preflight/RUN_FREEZE.json')
    require(sha(root/'PLAN.json')==freeze['plan_sha256'],'PLAN changed after freeze')
    for p,digest in freeze['files'].items():require(sha(Path(p))==digest,'Frozen file changed: '+p)
    return freeze


def binding_path(root,stage):return root/'preflight/stage_bindings'/(stage+'.json')


def bind_stage(root,stage,decision):
    plan=load_plan(root);freeze=verify_freeze(root);require(stage in ('bow','matrix'),'Only later stages need decisions')
    require_predecessors(root,plan,next(r for r in plan['runs'] if r['stage']==stage))
    require(bool(decision.get('decision_evidence')),'Stage decision evidence must be recorded')
    fields={k:decision[k] for k in ('package_id','observation','bow_ready') if k in decision}
    require(bool(fields),'No package/runtime decision fields')
    if 'package_id' in fields:require(fields['package_id'] in plan['packages'],'Decision references unknown package')
    for k in ('observation','bow_ready'):
        if k in fields:require(isinstance(fields[k],bool),'Decision switches must be booleans')
    for run in plan['runs']:
        if run['stage']!=stage:continue
        for key in ('package_id','observation','bow_ready'):
            if run.get(key) is None:require(key in fields,'Decision missing '+key)
            elif key in fields:require(fields[key]==run[key],'Decision conflicts with predeclared '+key)
    value=dict(schema='rgbd_stage_binding_v1',stage=stage,plan_sha256=freeze['plan_sha256'],created=now(),values=fields,decision_evidence=decision.get('decision_evidence',{}))
    write_json(binding_path(root,stage),value,exclusive=True)
    return value


def resolve_run(root,plan,run):
    result=dict(run);path=binding_path(root,run['stage'])
    if path.exists():
        binding=read_json(path);require(binding['plan_sha256']==sha(root/'PLAN.json'),'Stage binding PLAN mismatch')
        require(binding['stage']==run['stage'],'Wrong stage binding')
        for key,value in binding['values'].items():
            if result.get(key) is None:result[key]=value
            else:require(result[key]==value,'Stage binding conflicts with schedule')
        result['stage_binding_sha256']=sha(path)
    else:result['stage_binding_sha256']=None
    require(result.get('package_id') in plan['packages'],'Stage requires package binding: '+run['stage'])
    require(all(isinstance(result.get(k),bool) for k in ('observation','bow_ready')),'Stage requires runtime binding: '+run['stage'])
    result['package']=plan['packages'][result['package_id']]
    result['config']=plan['configs'][result['config_id']]
    result['resolved_identity_sha256']=object_sha(result)
    return result


def seal_run(root,resolved):
    run=root/'runs'/resolved['id']
    require(not (run/'RUN_COMPLETE.json').exists(),'Round already sealed')
    require(not (run/'REVIEWED_FAILURE.json').exists(),'Reviewed failure cannot be resealed as success')
    hashes={str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file() and p.name!='RUN_COMPLETE.json'}
    value=dict(schema='rgbd_completed_round_v1',run_id=resolved['id'],resolved_identity_sha256=resolved['resolved_identity_sha256'],created=now(),files=hashes)
    write_json(run/'RUN_COMPLETE.json',value,exclusive=True)
    return value


def verify_sealed_run(root,resolved):
    run=root/'runs'/resolved['id'];seal=read_json(run/'RUN_COMPLETE.json')
    require(seal['resolved_identity_sha256']==resolved['resolved_identity_sha256'],'Completed round identity differs from frozen stage')
    for name,digest in seal['files'].items():require(sha(run/name)==digest,'Completed round artifact changed: '+name)
    result=read_json(run/'RESULT.json');summary=read_json(run/'SUMMARY.json')
    require(result.get('complete') is True and summary.get('trial_valid') is True,'Completed round is invalid')
    return seal


def verify_failure_source_identity(root,run):
    """Missing observations may show overload; observed identity errors may not."""
    plan=load_plan(root)
    with local(root,plan['input']['index']).open() as f:index=list(csv.DictReader(f))
    expected={int(r['source_ordinal']):r for r in index}
    by_stamp={int(r['source_stamp_ns']):r for r in index}
    host_status=read_json(run/'host_events.status.json')
    completion=read_json(run/'board/timing/completion.json')
    require(all(host_status.get(k)==0 for k in ('identity_duplicate_or_overflow_errors','callback_errors')),'Host source identity/error counters cannot be waived')
    require(all(completion.get(k)==0 for k in ('identity_errors','callback_errors','cpu_clock_errors')),'Board source identity/error counters cannot be waived')
    with (run/'host_events.csv').open() as f:host=list(csv.DictReader(f))
    seen={'host_rgb':set(),'host_depth':set()}
    for row in host:
        kind=row['event'];original=by_stamp.get(int(row['source_stamp_ns']))
        require(kind in seen and original is not None,'Host source identity mismatch cannot be waived')
        key='rgb_seq' if kind=='host_rgb' else 'depth_seq'
        require(int(row['message_seq'])==int(original[key]) and int(row['rgb_seq'])==int(original['rgb_seq']),'Host source sequence identity mismatch cannot be waived')
        ordinal=int(original['source_ordinal'])
        require(ordinal not in seen[kind],'Host duplicate source identity cannot be waived')
        seen[kind].add(ordinal)
    require(host_status.get('records')==len(host),'Host observation record count mismatch cannot be waived')
    counts={kind:dict(observed=len(values),missing=len(expected)-len(values)) for kind,values in seen.items()}
    for name in ('scale_frames.csv','scale_sources.csv'):
        with (run/'board/timing'/name).open() as f:rows=list(csv.DictReader(f))
        observed=set()
        for row in rows:
            ordinal=int(row['source_ordinal']);original=expected.get(ordinal)
            require(original is not None and all(int(row[k])==int(original[k]) for k in ('source_stamp_ns','rgb_seq','depth_seq')),'Board '+name+' source identity mismatch cannot be waived')
            require(ordinal not in observed,'Board duplicate '+name+' source identity cannot be waived')
            require(all(int(row.get(k,0))==0 for k in ('callback_error','clock_failures')),'Board callback/clock error cannot be waived')
            observed.add(ordinal)
        counts[name]=dict(observed=len(observed),missing=len(expected)-len(observed))
    kinds=('received_rgb','received_depth','sync_drop_rgb','sync_drop_depth','paired','returned','valid_fresh')
    with (run/'board/timing/scale_events.csv').open() as f:events=list(csv.DictReader(f))
    for row in events:
        kind=row['kind'];original=expected.get(int(row['source_ordinal']))
        require(kind in kinds and original is not None,'Board event source identity mismatch cannot be waived')
        key='depth_seq' if kind in ('received_depth','sync_drop_depth') else 'rgb_seq'
        require(int(row['stamp_ns'])==int(original['source_stamp_ns']) and int(row['seq'])==int(original[key]),'Board event stamp/sequence identity mismatch cannot be waived')
    counts['scale_events.csv']=dict(observed_records=len(events))
    diagnostic=run/'board/stability/stability_frames.csv'
    if diagnostic.exists():
        with diagnostic.open() as f:rows=list(csv.DictReader(f))
        observed=set()
        for row in rows:
            ordinal=int(row['source_ordinal']);original=expected.get(ordinal)
            require(original is not None and all(int(row[k])==int(original[k]) for k in ('source_stamp_ns','rgb_seq','depth_seq')),'Diagnostic source identity mismatch cannot be waived')
            require(ordinal not in observed,'Diagnostic duplicate source identity cannot be waived');observed.add(ordinal)
        counts['stability_frames.csv']=dict(observed=len(observed),missing=len(expected)-len(observed))
    return dict(all_observed_source_identities_verified=True,expected_pairs=len(expected),observations=counts,
        scope='Missing inputs/board recorder overflow remain failed candidate evidence; identity, callback and clock errors are never waived. Host combined identity/duplicate/overflow counter must be zero because its causes cannot be separated.')


def review_failure(root,resolved,decision):
    """Explicit operator review may continue the fixed schedule, never rerun it."""
    run=root/'runs'/resolved['id'];result=read_json(run/'RESULT.json')
    require(not (run/'RUN_COMPLETE.json').exists(),'Successful round is immutable')
    require(decision.get('classification')=='candidate_overload' and decision.get('continue_predeclared_schedule') is True and bool(decision.get('reason')) and bool(decision.get('evidence')),'Failure review needs candidate_overload classification, reason, evidence and explicit continuation')
    require(result.get('phase')=='finished' and result.get('resolved_identity_sha256')==resolved['resolved_identity_sha256'],'Failure round identity/phase mismatch')
    require(result.get('transfer_verified') is True and all(result.get(k)==0 for k in ('slam_cleanup_exit','sampler_cleanup_exit')),'Unsafe cleanup/transfer cannot be waived')
    post=read_json(run/'board_postflight.json');host=read_json(run/'host_postflight.json');loaded=read_json(run/'loaded.json')
    require(post.get('sha256')==resolved['package']['binaries'] and post.get('all_owned_processes_exited') is True and not post.get('active') and not post.get('atlas_files'),'Board identity/residual-process failure cannot be waived')
    require(host.get('bindable') is True and host.get('owned_children_exited') is True,'Host cleanup failure cannot be waived')
    require(loaded.get('actual_sha256')==resolved['package']['binaries'] and loaded.get('settings_sha256')==resolved['config']['sha256'],'Actual binary/settings identity failure cannot be waived')
    require(loaded.get('eager_bow')==('1' if resolved['bow_ready'] else '0') and loaded.get('stability_directory')==result.get('stability_directory'),'Startup switch mismatch cannot be waived')
    for name,digest in read_json(run/'remote_sha256.json').items():require(sha(run/'board'/name)==digest,'Failure artifact transfer changed')
    source_identity=verify_failure_source_identity(root,run)
    hashes={str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file() and p.name!='REVIEWED_FAILURE.json'}
    value=dict(schema='rgbd_reviewed_candidate_failure_v1',run_id=resolved['id'],resolved_identity_sha256=resolved['resolved_identity_sha256'],created=now(),decision=decision,source_identity=source_identity,files=hashes,trial_valid=False,scope='Retained failed candidate round; explicit review permits later predeclared rounds only. Never transformed into a successful or repeat round.')
    write_json(run/'REVIEWED_FAILURE.json',value,exclusive=True)
    return value


def verify_finished_round(root,resolved):
    run=root/'runs'/resolved['id']
    if (run/'RUN_COMPLETE.json').exists():return verify_sealed_run(root,resolved)
    value=read_json(run/'REVIEWED_FAILURE.json')
    require(value.get('schema')=='rgbd_reviewed_candidate_failure_v1' and value.get('trial_valid') is False and value.get('resolved_identity_sha256')==resolved['resolved_identity_sha256'],'Unreviewed or mismatched failed round')
    for name,digest in value['files'].items():require(sha(run/name)==digest,'Reviewed failed-round artifact changed: '+name)
    return value


def require_predecessors(root,plan,run):
    for earlier in plan['runs']:
        if earlier['id']==run['id']:return
        verify_finished_round(root,resolve_run(root,plan,earlier))
    raise ValueError('Round absent from frozen schedule')
