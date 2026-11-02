#!/usr/bin/env python3
"""Offline reduction of all scheduled rounds, retaining failed and unrun rounds."""
import argparse
from collections import Counter
import math
from pathlib import Path
import re

import legacy_reduce as legacy
from trial_common import (DEFAULT_ROOT, BINARY_NAMES, load_plan, local, read_json,
                          resolve_run, sha, verify_freeze, verify_finished_round, write_json)


def original_entry(root,plan,resolved):
    inp=plan['input']
    with local(root,inp['index']).open() as stream:
        import csv
        ids=list(csv.DictReader(stream))
    stamps=[int(r['source_stamp_ns']) for r in ids]
    return dict(id=resolved['id'],bag=str(local(root,inp['bag'])),bag_sha256=inp['bag_sha256'],
        bag_bytes=inp['bag_bytes'],bag_mtime_ns=inp['bag_mtime_ns'],expected_pairs=5878,
        index_sha256=inp['index_sha256'],record_table_sha256=inp['records_sha256'],
        original_record_start_ns=inp['original_record_start_ns'],original_record_end_ns=inp['original_record_end_ns'],
        bag_record_duration_s=inp.get('bag_record_duration_s',(inp['original_record_end_ns']-inp['original_record_start_ns'])/1e9),
        header_mean_hz=(len(stamps)-1)*1e9/(stamps[-1]-stamps[0]),
        header_gap_ms=legacy.stats((b-a)/1e6 for a,b in zip(stamps,stamps[1:])),
        rgb_frames=5878,depth_frames=5878,unpaired_rgb=0,unpaired_depth=0)


def map_and_trajectory(frames,index,node_log):
    expected={int(r['source_ordinal']):r for r in index}
    by_n={int(r['source_ordinal']):r for r in frames}
    ordered=sorted(by_n);changes=[];resets=[];previous=None;epoch=-1;last_map=None;epoch_rows=[]
    for n in ordered:
        r=by_n[n];map_id=int(r['map_id']);snapshot=int(r['scale_snapshot_mono_ns'])>=0 and map_id>=0
        fresh=r['pose_valid']==r['pose_fresh']=='1'
        reset=(previous is not None and n==int(previous['source_ordinal'])+1 and previous['state_after']=='4' and r['state_after']=='2' and r['global_kf']==r['global_mp']=='0' and map_id==int(previous['map_id']))
        change=snapshot and last_map is not None and map_id!=last_map
        if change:changes.append(dict(source_ordinal=n,from_map=last_map,to_map=map_id))
        if reset:resets.append(dict(request_ordinal=n-1,next_initialization_ordinal=n,map_id=map_id))
        if snapshot and (epoch<0 or change or reset):
            epoch+=1;epoch_rows.append(dict(epoch=epoch,map_id=map_id,start_ordinal=n))
        if snapshot:last_map=map_id
        r['_coordinate_epoch']=epoch if snapshot else None
        previous=r
    log_lines=node_log.splitlines();creation=[];reset_logs=[];merge_logs=[]
    for line_no,line in enumerate(log_lines,1):
        match=re.search(r'Creation of new map with id:\s*(\d+)',line)
        if match:creation.append(dict(line=line_no,map_id=int(match.group(1))))
        if 'LM: Active map reset, Done!!!' in line:reset_logs.append(line_no)
        if re.search(r'merge detected|merg(?:ing|ed) maps|map merged|merge (?:finished|complete)',line,re.I):merge_logs.append(dict(line=line_no,text=line))
    steps=[]
    for a,b in zip(ordered,ordered[1:]):
        ra,rb=by_n[a],by_n[b]
        if b!=a+1 or ra['_coordinate_epoch'] is None or ra['_coordinate_epoch']!=rb['_coordinate_epoch']:continue
        if not (ra['pose_valid']==ra['pose_fresh']==rb['pose_valid']==rb['pose_fresh']=='1'):continue
        xyz_a=[float(ra[k]) for k in ('tx','ty','tz')];xyz_b=[float(rb[k]) for k in ('tx','ty','tz')]
        if all(math.isfinite(v) for v in xyz_a+xyz_b):steps.append(math.sqrt(sum((x-y)**2 for x,y in zip(xyz_a,xyz_b))))
    stamps={n:int(r['source_stamp_ns']) for n,r in expected.items()};last=max(expected)
    nonfresh_s=fresh_s=unknown_s=0.
    for n in sorted(expected):
        if n==last:continue
        dt=(stamps[n+1]-stamps[n])/1e9;r=by_n.get(n)
        if r is None:unknown_s+=dt
        elif r['pose_valid']==r['pose_fresh']=='1':fresh_s+=dt
        else:nonfresh_s+=dt
    fresh_runs=[];current=[]
    for n in sorted(expected):
        r=by_n.get(n);good=r is not None and r['pose_valid']==r['pose_fresh']=='1'
        if good:current.append(n)
        elif current:fresh_runs.append(current);current=[]
    if current:fresh_runs.append(current)
    durations=[(stamps[g[-1]]-stamps[g[0]])/1e9 for g in fresh_runs]
    tail=by_n.get(last)
    return dict(map_id_changes=dict(count=len(changes),events=changes,scope='Observed ID changes; not all changes are new-map creations'),
        new_map_creation_logs=dict(observed_events=creation,after_initial_atlas_creation=max(0,len(creation)-1) if creation else None,scope='Log-based counts; not source-timestamped. No log matches means unknown, not zero.'),
        same_id_active_resets=dict(raw_next_initialization_hints=resets,raw_count=len(resets),completed_log_lines=reset_logs,completed_log_count=len(reset_logs),counts_agree=len(resets)==len(reset_logs)),
        merge_events_unlocalized=merge_logs,coordinate_epochs=epoch_rows,
        time_weights=dict(nonfresh_s=nonfresh_s,fresh_s=fresh_s,unknown_unprocessed_s=unknown_s,source_header_span_s=(stamps[last]-stamps[1])/1e9,eof_last_frame_nonfresh=bool(tail and not(tail['pose_valid']==tail['pose_fresh']=='1')),method='Assign each known input state to next original header delta; last input right-censored, never fabricate its duration'),
        contiguous_fresh=dict(segments=len(fresh_runs),length_frames=legacy.stats(len(g) for g in fresh_runs),first_to_last_header_s=legacy.stats(durations),longest_frames=max(map(len,fresh_runs),default=0)),
        trajectory=dict(fresh_finite_translation_steps_same_epoch_consecutive_sources=None if merge_logs else legacy.stats(steps),candidate_pairs=len(steps),step_statistics_suppressed_due_to_unlocalized_merge=bool(merge_logs),scope='Raw Twc within distinct coordinate epochs, consecutive original ordinals and fresh finite poses only. Cross-map/reset/merge paths are never joined. No GT or trajectory-error qualification; an apparent step is not proof of wrong pose.'))


def reduce_one(root,plan,resolved):
    run=root/'runs'/resolved['id'];result=read_json(run/'RESULT.json');pkg=resolved['package'];cfg=resolved['config']
    view=dict(package=pkg['remote_path'],source_commit=pkg['source_commit'],settings_sha256=cfg['sha256'],rate=1.,queues=[8,8,10])
    prior=legacy.BINARIES
    try:
        legacy.BINARIES=pkg['binaries']
        reduced=legacy.reduce_run(root,view,original_entry(root,plan,resolved),result)
    finally:legacy.BINARIES=prior
    problems=reduced['problems']
    if any(reduced['board_unique_source_counts'].get(k)!=5878 for k in ('received_rgb','received_depth','paired','returned')):problems.append('Board input identity sets do not cover all5878 sources')
    with (run/'board/timing/scale_frames.csv').open() as f:
        import csv
        frames=list(csv.DictReader(f))
    with (root/'inputs'/resolved['id']/'input_index.csv').open() as f:index=list(csv.DictReader(f))
    for kind,consumer in reduced['consumers'].items():
        if not consumer['all_fresh_sources_observed'] or consumer['duplicate_stamps'] or consumer['extra_stamps_without_frame']:problems.append(kind+' pose identity/fresh completeness failure')
    pose_status=read_json(run/'poses/status.json') if (run/'poses/status.json').exists() else {}
    if not pose_status.get('normal_spin_return') or pose_status.get('errors'):problems.append('pose observer did not exit cleanly')
    if result.get('pose_observer_exit')!=0 or result.get('sampler_exit')!=0:problems.append('sampler or pose observer nonzero/missing exit')
    if result.get('resolved_identity_sha256')!=resolved['resolved_identity_sha256']:problems.append('RESULT differs from resolved frozen run identity')
    post=read_json(run/'board_postflight.json') if (run/'board_postflight.json').exists() else {}
    if post.get('sha256')!=pkg['binaries'] or not post.get('all_owned_processes_exited') or post.get('active') or post.get('atlas_files'):problems.append('post-run package/process/Atlas audit failed')
    loaded=read_json(run/'loaded.json')
    if loaded.get('actual_sha256')!=pkg['binaries']:problems.append('Actual loaded exe/core content hashes differ from package manifest')
    if loaded.get('eager_bow')!=('1' if resolved['bow_ready'] else '0'):problems.append('Actual startup BoW switch differs from frozen run')
    expected_dir=result.get('stability_directory') if resolved['observation'] else None
    if loaded.get('stability_directory')!=expected_dir:problems.append('Actual startup observation directory differs from frozen run')
    diag_path=run/'board/stability/stability_summary.json'
    diagnostic=read_json(diag_path) if diag_path.exists() else None
    if resolved['observation'] and diagnostic is None:problems.append('Observation enabled but diagnostic summary missing')
    if not resolved['observation'] and diagnostic is not None:problems.append('Observation disabled but diagnostic output exists')
    diagnostic_frames_valid=None
    if diagnostic is not None:
        import csv
        frame_path=run/'board/stability/stability_frames.csv'
        diagnostic_frames=list(csv.DictReader(frame_path.open())) if frame_path.exists() else []
        diagnostic_frames_valid=(diagnostic.get('schema')=='rgbd_stability_v1' and diagnostic.get('complete') is True and diagnostic.get('frames')==5878 and diagnostic.get('frame_drops')==0 and len(diagnostic_frames)==5878 and all(
            all(str(a.get(k))==str(b[k]) for k in ('source_ordinal','source_stamp_ns','rgb_seq','depth_seq')) for a,b in zip(diagnostic_frames,index)))
        if not diagnostic_frames_valid:problems.append('Diagnostic frame identities/counters/files incomplete')
        if bool(diagnostic.get('eager_bow'))!=resolved['bow_ready']:problems.append('Diagnostic runtime BoW mode mismatch')
    reduced.update(schema='rgbd_stability_round_summary_v1',trial_id=resolved['id'],stage=resolved['stage'],config_id=resolved['config_id'],package_id=resolved['package_id'],observation=resolved['observation'],bow_ready=resolved['bow_ready'],resolved_identity_sha256=resolved['resolved_identity_sha256'],
        dynamics=map_and_trajectory(frames,index,(run/'node.log').read_text(errors='replace')),
        diagnostic=diagnostic,diagnostic_frames_valid=diagnostic_frames_valid,diagnostic_lossless_all_streams=(diagnostic.get('workers_joined') is True and all(diagnostic.get(k)==0 for k in ('frame_drops','point_drops','lifetime_drops','closed_appends_at_flush'))) if diagnostic else None,
        diagnostic_shutdown_scope='Diagnostic complete refers only to three diagnostic CSV flushes, never inferred backend/thread join. workers_joined=false is retained as an observation limitation, not silently promoted.',
        diagnostic_schema_scope='Diagnostic counters retained verbatim, absent values remain absent; bounded point/lifetime evidence and drops must be inspected separately.',
        pose_observer_status=pose_status,board_postflight=post,
        trial_valid=reduced['execution_complete'] and reduced['paired_input_complete'] and not problems,
        measurement_integrity_ok=not problems)
    return reduced


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,default=DEFAULT_ROOT)
    parser.add_argument('--stage',choices=('observation','bow','matrix'));args=parser.parse_args();root=args.root.resolve();plan=load_plan(root);verify_freeze(root)
    records=[]
    for run in plan['runs']:
        if args.stage and run['stage']!=args.stage:continue
        path=root/'runs'/run['id']/'RESULT.json'
        if not path.exists():records.append(dict(id=run['id'],stage=run['stage'],status='not_run'));continue
        result=read_json(path)
        if result.get('phase')!='finished':records.append(dict(id=run['id'],stage=run['stage'],status='incomplete',result=result));continue
        try:
            resolved=resolve_run(root,plan,run);summary=reduce_one(root,plan,resolved)
            reviewed=(root/'runs'/run['id']/'REVIEWED_FAILURE.json').exists()
            if reviewed:verify_finished_round(root,resolved)
            records.append(dict(id=run['id'],stage=run['stage'],status='reviewed_candidate_failure' if reviewed else ('valid' if summary['trial_valid'] else 'invalid'),summary=summary))
        except Exception as error:
            reviewed=(root/'runs'/run['id']/'REVIEWED_FAILURE.json').exists()
            if reviewed:
                verify_finished_round(root,resolve_run(root,plan,run))
            records.append(dict(id=run['id'],stage=run['stage'],status='reviewed_candidate_failure' if reviewed else 'invalid',error=repr(error),result=result))
    suffix='_'+args.stage if args.stage else ''
    value=dict(schema='rgbd_stability_matrix_summary_v1',planned_rounds=16,selected_stage=args.stage,status_counts=dict(Counter(r['status'] for r in records)),rounds=records,scope='All planned/failed rounds retained. No automatic winner, no GT precision or backend-join inference. Independent stage decisions use frozen PROTOCOL thresholds.')
    write_json(root/('SUMMARY'+suffix+'.json'),value)
    print(str(root/('SUMMARY'+suffix+'.json')))
    return 0 if all(r['status'] in ('valid','not_run','reviewed_candidate_failure') for r in records) else 2


if __name__=='__main__':raise SystemExit(main())
