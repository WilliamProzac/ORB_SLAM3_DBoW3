#!/usr/bin/env python3
"""Create the16-round PLAN once from completed build and preserved input metadata."""
import argparse
from pathlib import Path
from trial_common import DEFAULT_ROOT, MATRIX, TOOLS, read_json, require, sha, write_json, load_plan


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=DEFAULT_ROOT)
    parser.add_argument('--original',type=Path)
    parser.add_argument('--configs-dir',type=Path,default=TOOLS.parent/'configs')
    parser.add_argument('--build-result',type=Path,required=True)
    parser.add_argument('--source-commit',required=True)
    parser.add_argument('--package-id',default='runtime')
    parser.add_argument('--package',required=True,help='Already deployed remote node/core package path')
    parser.add_argument('--shared-lib-dir',default='/userdata/rgbd_tracking_optimizations_20261020/atomic_02/lib')
    parser.add_argument('--vocabulary',default='/userdata/rgbd_tracking_optimizations_20261020/atomic_02/Vocabulary/orbvoc.dbow3')
    parser.add_argument('--helpers-dir',default='/userdata/rgbd_tracking_optimizations_20261020/atomic_02/tools')
    args=parser.parse_args();root=args.root.resolve();original=(args.original or root.parent/'rgbd_d435i_003015_20261030').resolve()
    old=read_json(original/'PLAN.json');entry=old['runs'][0];build=read_json(args.build_result)
    require(build.get('build_complete') is True,'Build must be complete before freezing package hashes')
    require(set(build['binaries'])=={'ros_rgbd','liborb_slam3_ros.so'},'Completed matched node/core hashes required')
    source_input=original/'inputs'/entry['id']
    inp={k:entry[k] for k in ('bag','bag_sha256','bag_bytes','bag_mtime_ns','expected_pairs','original_record_start_ns','original_record_end_ns','bag_record_duration_s')}
    inp.update(index=str(source_input/'input_index.csv'),index_sha256=entry['index_sha256'],records=str(source_input/'original_input_records.csv'),records_sha256=entry['record_table_sha256'],rgb_topic=old['rgb_topic'],depth_topic=old['depth_topic'])
    configs={}
    for key,features,levels in [('P0',700,5),('P1',1000,5),('P2',700,8),('P3',1000,8)]:
        path=(args.configs_dir/('%s_%d_%d.yaml'%(key,features,levels))).resolve();configs[key]=dict(path=str(path),sha256=sha(path),features=features,levels=levels)
    package=dict(remote_path=args.package,binaries=build['binaries'],source_commit=args.source_commit,
        shared_lib_dir=args.shared_lib_dir,vocabulary=args.vocabulary,helpers_dir=args.helpers_dir,
        build_result=str(args.build_result.resolve()),source_ready_sha256=build.get('source_ready_sha256'))
    manifest=args.build_result.parent/'manifest.json'
    if manifest.exists():package['build_manifest']=str(manifest.resolve())
    runs=[]
    for stage,prefix in [('observation','B_obs'),('bow','B_bow')]:
        for n,on in enumerate([False,True,True,False],1):
            runs.append(dict(id='%s_%02d_%s'%(prefix,n,'on' if on else 'off'),stage=stage,package_id=args.package_id,config_id='P0',observation=on if stage=='observation' else None,bow_ready=False if stage=='observation' else on))
    for n,config in enumerate(MATRIX,1):runs.append(dict(id='C_%02d_%s'%(n,config),stage='matrix',package_id=None,config_id=config,observation=None,bow_ready=None))
    plan=dict(schema='rgbd_stability_trials_v1',feature='RGBD36',input=inp,packages={args.package_id:package},configs=configs,runs=runs,
        rate=1.,queues=[8,8,10],network=dict(board='sunrise@192.168.1.11',board_ip='192.168.1.11',host_ip='192.168.1.3',master_port=11431),
        remote_root='/userdata/rgbd_stability_trials_20261102',min_board_free_bytes=1024**3,
        diagnostic_windows='23-100,640-710,1320-1420,2180-2270,3380-3470,5000-5220',
        protocol_file='preflight/EXPERIMENT_PROTOCOL.json',scope='Input-complete original1x repeated trials. Stage decisions are immutable separate bindings; no automatic repeat, candidate winner or GT precision claim.')
    write_json(root/'PLAN.json',plan,exclusive=True);load_plan(root)
    print(str(root/'PLAN.json'))


if __name__=='__main__':main()
