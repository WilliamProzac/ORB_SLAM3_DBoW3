#!/usr/bin/env python3
import argparse, json
from pathlib import Path
import numpy as np
from trajectory_se3 import read_poses, associate, evaluate

ROOT=Path(__file__).resolve().parents[1]
def alignment(ep,gp,scale_free):
    ec,gc=ep.mean(axis=0),gp.mean(axis=0)
    x,y=ep-ec,gp-gc
    u,d,vt=np.linalg.svd(x.T@y)
    sign=np.eye(3);sign[2,2]=np.linalg.det(vt.T@u.T)
    R=vt.T@sign@u.T
    s=float(np.sum(d*np.diag(sign))/np.sum(x*x)) if scale_free else 1.
    return s,R,gc-s*R@ec

def accuracy(path,dataset):
    est=read_poses(path)
    truthpath=ROOT/'ORB_Data/EuRoC_POSE_GT/V1_01_easy_cam0.txt' if dataset=='V1_01_easy' else Path('/home/ywl/dataset/rgbd_dataset_freiburg2_large_with_loop/groundtruth.txt')
    truth=read_poses(truthpath);pairs=associate(est[0],truth[0],.02)
    if len(pairs)<3:raise ValueError('insufficient associations')
    ei,gi=np.array(pairs).T
    s,R,t=alignment(est[1][ei],truth[1][gi],dataset=='V1_01_easy')
    aligned=(est[0],s*est[1]@R.T+t,R@est[2])
    result=evaluate(aligned,truth)
    result.update(alignment='Sim3 (one global scale)' if dataset=='V1_01_easy' else 'SE3 (scale 1)',scale=s,alignment_rotation=R.tolist(),alignment_translation=t.tolist(),truth_path=str(truthpath),truth_max_gap_s=float(np.max(np.diff(truth[0]))),associated_fraction_of_output=len(pairs)/len(est[0]))
    return result

def stats(x):
    if not len(x):return None
    return dict(zip(['mean','p25','p50','p75','p95','p99','max'],[float(np.mean(x)),*map(float,np.percentile(x,[25,50,75,95,99,100]))]))

def summarize(run):
    meta=json.loads((run/'run.json').read_text());f=run/'frames.csv'
    x=np.atleast_1d(np.genfromtxt(f,names=True,delimiter=',')) if f.exists() else np.array([])
    out={k:meta[k] for k in ['mode','dataset','repeat','exit_code','elapsed_s','failure_reason','expected_frames']};out['run']=str(run)
    out['processed']=len(x);out['complete']=len(x)==meta['expected_frames'] and meta['exit_code']==0
    if not len(x):return out
    out['indices_contiguous']=bool(np.array_equal(x['index'],np.arange(len(x))))
    out['timestamps_increasing']=bool(np.all(np.diff(x['timestamp'])>0))
    out['finite_fields']=bool(all(np.isfinite(x[key]).all() for key in x.dtype.names))
    out['complete']=out['complete'] and out['indices_contiguous'] and out['timestamps_increasing'] and out['finite_fields']
    ok=np.flatnonzero(x['state']==2);start=int(ok[0]) if len(ok) else len(x)
    out['nominal_period_ms']=1000/(20 if meta['dataset']=='V1_01_easy' else 30)
    out['track_calls_over_nominal_period_after_init']=int(np.sum(x['track_ms'][start:]>out['nominal_period_ms']))
    out['first_ok_index']=start;out['non_ok_after_initialization']=int(np.sum(x['state'][start:]!=2));out['ok_frames']=len(ok)
    bad=np.flatnonzero((x['state']!=2)&(x['index']>=start))
    chunks=np.split(bad,np.flatnonzero(np.diff(bad)>1)+1) if len(bad) else []
    out['non_ok_ranges_inclusive']=[[int(c[0]),int(c[-1])] for c in chunks]
    out['tracking_pass']=out['complete'] and bool(len(ok)) and out['non_ok_after_initialization']==0
    out['states']={str(int(k)):int(v) for k,v in zip(*np.unique(x['state'],return_counts=True))}
    for label,subset in [('all',x),('after_initialization',x[start:]),('tail25',x[x['index']>=meta['expected_frames']-meta['expected_frames']//4])]:
        out[label]={key:stats(subset[key]) for key in ['track_ms','local_ms','match_ms','opt_ms','hash_field_ms','local_mp','covis_mp','hash_mp','inliers','pose_stage_sum_ms','post_ms','pose_ready_ms','gfm_active','gfm_matrix_ms','gfm_skip_local','valid_associations'] if key in subset.dtype.names}
    out['hash_query_nonzero_frames']=int(np.sum(x['hash_mp']>0));out['features_ok']=stats(x['features'][x['state']==2])
    console=(run/'console.log').read_text(errors='replace')
    out['module_execution']={
        'gfm_active_frames':int(np.sum(x['gfm_active']>0)),
        'gfm_skip_local_frames':int(np.sum(x['gfm_skip_local']>0)),
        'good_graph_triggered':console.count('Good Graph Triggered -------------'),
        'good_graph_completed':console.count('logDet(S) = '),
    }
    ots=[]
    for line in (run/'console.log').read_text(errors='replace').splitlines():
        if line.startswith('REPRO_OTS '):
            try:
                _,frame,completed,count,elapsed=line.split();ots.append((int(frame),int(completed),int(count),float(elapsed)))
            except ValueError:pass
    out['ots']={'updates':len(ots),'completed':sum(z[1] for z in ots),'table_counts':stats(np.array([z[2] for z in ots])),'update_ms':stats(np.array([z[3]*1000 for z in ots]))}
    resources=json.loads((run/'resources.json').read_text())
    out['resources']={'rss_peak_kib':max([int(z.get('VmRSS','0 kB').split()[0]) for z in resources] or [0]),'swap_peak_kib':max([int(z.get('VmSwap','0 kB').split()[0]) for z in resources] or [0])}
    cpu=[z for z in resources if 'cpu_ticks' in z]
    if len(cpu)>1:out['resources']['mean_process_cores']=(cpu[-1]['cpu_ticks']-cpu[0]['cpu_ticks'])/cpu[-1]['clock_ticks_per_second']/(cpu[-1]['elapsed_s']-cpu[0]['elapsed_s'])
    mapping=run/'upstream_mapping.txt'
    if mapping.exists():
        try:
            m=np.loadtxt(mapping,ndmin=2)
            if m.shape[1]!=19:raise ValueError('mapping column count differs from audited source')
            valid=m[:,5]>=0
            out['good_graph_times']={k:stats(m[valid,c]*1000) for k,c in [('insert',7),('jacobian',8),('random_query',9),('schur',10),('permutation',11),('cholesky',12),('post',13),('optimization',14)]} if meta.get('module_G') else {}
            out['local_ba']={'mapping_records':len(m),'timed_calls':int(np.sum(valid)),'wall_ms':stats(m[valid,5]*1000),'free_kf':stats(m[valid,17]),'fixed_kf':stats(m[valid,16]),'points':stats(m[valid,18])}
        except ValueError as e:out['mapping_parse_error']=str(e)
    out['final_kfs']=int(x['kfs'][-1]);out['final_mps']=int(x['mps'][-1])
    try:out['accuracy']=accuracy(run/'online.txt',meta['dataset'])
    except (ValueError,OSError) as e:out['accuracy_error']=str(e)
    if (run/'index_diagnostics.json').exists():
        out['index_diagnostics']=json.loads((run/'index_diagnostics.json').read_text())
        if meta['mode'] in ('paper_ots','paper_all'):
            out['index_storage_bound_pass']=out['index_diagnostics']['max_bucket']<=10 and out['index_diagnostics']['entries']<=32*256*10
    out['accuracy_eligible']=out['tracking_pass'] and 'accuracy' in out
    return out

def main():
    p=argparse.ArgumentParser();p.add_argument('matrix',type=Path);a=p.parse_args()
    results=[summarize(f.parent) for f in sorted(a.matrix.glob('*/run.json'))]
    groups={}
    for mode in sorted({r['mode'] for r in results}):
        rr=[r for r in results if r['mode']==mode];g={'runs':len(rr),'tracking_pass_runs':sum(r.get('tracking_pass',False) for r in rr)}
        g['all_runs_tracking_pass']=bool(rr) and all(r.get('tracking_pass',False) for r in rr)
        g['timing_includes_failed_runs']=not g['all_runs_tracking_pass']
        for key in ['mean','p50','p95','p99']:
            g['median_run_track_'+key]=float(np.median([r['after_initialization']['track_ms'][key] for r in rr if r.get('after_initialization',{}).get('track_ms')]))
        for key in ['mean','p95','p99']:
            g['median_run_pose_ready_'+key]=float(np.median([r['after_initialization']['pose_ready_ms'][key] for r in rr if r.get('after_initialization',{}).get('pose_ready_ms')]))
        g['median_run_tail_p95']=float(np.median([r['tail25']['track_ms']['p95'] for r in rr if r.get('tail25',{}).get('track_ms')]))
        eligible=[r for r in rr if r.get('accuracy_eligible')];g['valid_accuracy_runs']=len(eligible)
        if eligible:g['median_valid_ate_rmse_m']=float(np.median([r['accuracy']['ate_rmse_m'] for r in eligible]))
        for key in ['rpe_translation_rmse_m','rpe_rotation_rmse_deg']:
            if eligible:g['median_valid_'+key]=float(np.median([r['accuracy'][key] for r in eligible]))
        g['module_execution_totals']={key:sum(r.get('module_execution',{}).get(key,0) for r in rr) for key in ['gfm_active_frames','gfm_skip_local_frames','good_graph_triggered','good_graph_completed']}
        g['ots_update_count']=sum(r.get('ots',{}).get('updates',0) for r in rr)
        for name in ['valid_associations','local_ms','local_mp','match_ms','post_ms']:
            vals=[r['after_initialization'][name]['mean'] for r in rr if r.get('after_initialization',{}).get(name)]
            if vals:g['median_run_'+name+'_mean']=float(np.median(vals))
        groups[mode]=g
    out={'runs':results,'groups':groups,'scope':'host native Track API, decoding excluded, no frame drop, no loop closure; failed runs retained'}
    (a.matrix/'analysis.json').write_text(json.dumps(out,indent=2));print(json.dumps(groups,indent=2))
if __name__=='__main__':main()
