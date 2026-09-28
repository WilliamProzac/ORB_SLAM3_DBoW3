#!/usr/bin/env python3
import csv,json
from pathlib import Path
import numpy as np
from trajectory_se3 import read_poses,associate,evaluate
from run import ROOT,GF,sha

def stats(x):
    if not len(x):return None
    return dict(zip(['mean','p50','p95','p99','max'],map(float,[np.mean(x),*np.percentile(x,[50,95,99,100])])))
def alignment(ep,gp,scale_free):
    ec,gc=ep.mean(axis=0),gp.mean(axis=0);x,y=ep-ec,gp-gc
    u,d,vt=np.linalg.svd(x.T@y);sign=np.eye(3);sign[2,2]=np.linalg.det(vt.T@u.T)
    R=vt.T@sign@u.T;s=float(np.sum(d*np.diag(sign))/np.sum(x*x)) if scale_free else 1.
    return s,R,gc-s*R@ec
def accuracy(path,dataset):
    est=read_poses(path)
    truthpath=GF/'ORB_Data/EuRoC_POSE_GT/V1_01_easy_cam0.txt' if dataset=='V1_01_easy' else Path('/home/ywl/dataset/rgbd_dataset_freiburg2_large_with_loop/groundtruth.txt')
    truth=read_poses(truthpath);pairs=associate(est[0],truth[0],.02)
    if len(pairs)<3:raise ValueError('Insufficient associated poses')
    ei,gi=np.array(pairs).T;ep,gp=est[1][ei],truth[1][gi]
    s,R,t=alignment(ep,gp,dataset=='V1_01_easy')
    result=evaluate((est[0],s*est[1]@R.T+t,R@est[2]),truth)
    result.update(alignment='global Sim3' if dataset=='V1_01_easy' else 'global SE3',scale=s,truth_path=str(truthpath),truth_sha256=sha(truthpath),truth_max_gap_s=float(np.max(np.diff(truth[0]))),associated_fraction_of_output=len(pairs)/len(est[0]))
    return result
def summarize(run):
    meta=json.loads((run/'run.json').read_text());x=np.atleast_1d(np.genfromtxt(run/'frames.csv',names=True,delimiter=','))
    out={k:meta[k] for k in ['mode','dataset','exit_code','elapsed_s','failure_reason','expected_frames']}
    out['run']=str(run);out['processed']=len(x);out['schema']=list(x.dtype.names)
    out['finite']=bool(all(np.isfinite(x[k]).all() for k in x.dtype.names))
    out['indices_contiguous']=bool(np.array_equal(x['index'],np.arange(len(x))))
    out['timestamps_increasing']=bool(np.all(np.diff(x['timestamp'])>0))
    console=(run/'console.log').read_text(errors='replace')
    out['completion_marker']=f"REPRO_COMPLETE {meta['expected_frames']}" in console
    out['complete']=len(x)==meta['expected_frames'] and meta['exit_code']==0 and out['finite'] and out['indices_contiguous'] and out['timestamps_increasing'] and out['completion_marker']
    ok=np.flatnonzero(x['state']==2);start=int(ok[0]) if len(ok) else len(x)
    out['first_ok_index']=start;out['non_ok_after_initialization']=int(np.sum(x['state'][start:]!=2))
    out['states']={str(int(k)):int(v) for k,v in zip(*np.unique(x['state'],return_counts=True))}
    bad=np.flatnonzero((x['index']>=start)&(x['state']!=2));chunks=np.split(bad,np.flatnonzero(np.diff(bad)>1)+1) if len(bad) else []
    out['non_ok_ranges']=[[int(c[0]),int(c[-1])] for c in chunks]
    out['map_ids']=list(map(int,np.unique(x['map_id']))) if 'map_id' in x.dtype.names else None
    out['max_maps']=int(max(x['maps'])) if 'maps' in x.dtype.names else None
    out['tracking_pass']=out['complete'] and len(ok)>0 and out['non_ok_after_initialization']==0 and (out['map_ids'] is None or len(out['map_ids'])==1)
    period=1000/(20 if meta['dataset']=='V1_01_easy' else 30)
    out['period_ms']=period
    for name,part in [('all',x),('after_initialization',x[start:]),('ok_only',x[x['state']==2]),('tail25',x[x['index']>=meta['expected_frames']-meta['expected_frames']//4])]:
        out[name]={k:stats(part[k]) for k in ['track_ms','input_lateness_ms','features','valid_associations','kfs','mps']}
        out[name]['over_period']=int(np.sum(part['track_ms']>period))
        out[name]['frames']=len(part)
    out['input_lateness_end_ms']=float(x['input_lateness_ms'][-1])
    out['final_kfs']=int(x['kfs'][-1]);out['final_mps']=int(x['mps'][-1])
    samples=json.loads((run/'resources.json').read_text())
    out['resources']={k:max([int(v.get(key,'0 kB').split()[0]) for v in samples] or [0]) for k,key in [('rss_peak_kib','VmHWM'),('swap_peak_kib','VmSwap'),('threads_peak','Threads')]}
    if len(samples)>1:out['resources']['mean_process_cores']=(samples[-1]['cpu_ticks']-samples[0]['cpu_ticks'])/samples[-1]['clock_ticks_per_second']/(samples[-1]['elapsed_s']-samples[0]['elapsed_s'])
    try:out['accuracy']=accuracy(run/'online.txt',meta['dataset'])
    except (OSError,ValueError) as e:out['accuracy_error']=str(e)
    out['accuracy_eligible']=out['tracking_pass'] and 'accuracy' in out
    out['delivery_workers_confirmed']='- Parallel Workers: 3' in console
    out['delivery_l4_confirmed']=('RGB-D exact optimization level: 4' in console) if meta['dataset']=='tum_large' and meta['mode'].startswith('delivered') else None
    return out
def main():
    results=[summarize(p.parent) for p in sorted((ROOT/'runs/formal').glob('*/*/run.json'))]
    # Different implementations initialize at different frames. Compare an
    # identical index range as well as their own post-initialization statistics.
    for ds in ['V1_01_easy','tum_large']:
        rr=[r for r in results if r['dataset']==ds]
        if not rr:continue
        cutoff=max(r['first_ok_index'] for r in rr)
        for r in rr:
            x=np.atleast_1d(np.genfromtxt(Path(r['run'])/'frames.csv',names=True,delimiter=','))
            part=x[x['index']>=cutoff]
            r['common_start_index']=cutoff
            r['common_range']={k:stats(part[k]) for k in ['track_ms','input_lateness_ms']}
            r['common_range']['frames']=len(part)
    previous=Path('/home/ywl/Project/ORB_SLAM3_DBoW3/artifacts/gf_orb_slam2_modules_20260928')
    history=[];bridge=[]
    for dataset,folder in [('V1_01_easy','euroc_modules'),('tum_large','tum_modules')]:
        old=json.loads((previous/folder/'analysis.json').read_text())['runs']
        official=next((r for r in results if r['dataset']==dataset and r['mode']=='official800'),None)
        current=next((r for r in results if r['dataset']==dataset and r['mode']=='gf_bridge'),None)
        baseline=next(r for r in old if r['mode']=='baseline')
        if current:
            bridge.append({'dataset':dataset,**{f'{k}_change_percent':100*(current['after_initialization']['track_ms'][k]/baseline['after_initialization']['track_ms'][k]-1) for k in ['mean','p95','p99']}})
        if official:
            for r in old:
                p95=r['after_initialization']['track_ms']['p95']
                history.append({'dataset':dataset,'mode':r['mode'],'historical':True,'track_p95_ms':p95,'official_p95_ms':official['after_initialization']['track_ms']['p95'],'change_vs_official_percent':100*(p95/official['after_initialization']['track_ms']['p95']-1),'tracking_pass':r['tracking_pass'],'ate_rmse_m':r.get('accuracy',{}).get('ate_rmse_m')})
    (ROOT/'analysis.json').write_text(json.dumps({'runs':results,'bridge':bridge,'historical_GF03_reference':history},indent=2)+'\n')
    rows=[]
    for r in results:
        t=r['after_initialization']['track_ms'];tail=r['tail25']['track_ms'];a=r.get('accuracy',{})
        rows.append(dict(dataset=r['dataset'],mode=r['mode'],processed=r['processed'],non_ok=r['non_ok_after_initialization'],map_count=r['max_maps'],mean_ms=t['mean'],p95_ms=t['p95'],p99_ms=t['p99'],max_ms=t['max'],tail25_p95_ms=tail['p95'],common_p95_ms=r['common_range']['track_ms']['p95'],late_p95_ms=r['after_initialization']['input_lateness_ms']['p95'],over_period=r['after_initialization']['over_period'],ate_m=a.get('ate_rmse_m'),accuracy_eligible=r['accuracy_eligible']))
    if rows:
        with (ROOT/'summary.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=rows[0].keys(),lineterminator='\n');w.writeheader();w.writerows(rows)
    print(json.dumps(rows,indent=2));print('BRIDGE',json.dumps(bridge))
if __name__=='__main__':main()
