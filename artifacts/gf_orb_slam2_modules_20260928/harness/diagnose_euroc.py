#!/usr/bin/env python3
"""Locate observed errors with the same full-run alignment; no segment realignment."""
import csv
import json
from pathlib import Path
import numpy as np
from trajectory_se3 import read_poses, associate

ROOT=Path(__file__).resolve().parents[1]

def main():
    data=json.loads((ROOT/'runs/euroc_modules/analysis.json').read_text())
    output={}
    points=[]
    for run in data['runs']:
        a=run['accuracy']; est=read_poses(Path(run['run'])/'online.txt'); truth=read_poses(Path(a['truth_path']))
        ei,gi=np.array(associate(est[0],truth[0],a['timestamp_tolerance_s'])).T
        aligned=a['scale']*est[1]@np.asarray(a['alignment_rotation']).T+np.asarray(a['alignment_translation'])
        err=np.linalg.norm(aligned[ei]-truth[1][gi],axis=1)
        assert abs(float(np.sqrt(np.mean(err**2)))-a['ate_rmse_m'])<1e-8
        frames=np.genfromtxt(Path(run['run'])/'frames.csv',names=True,delimiter=',')
        fi=np.searchsorted(frames['timestamp'],est[0][ei])
        assert np.allclose(frames['timestamp'][fi],est[0][ei],rtol=0,atol=1e-7)
        quarters=[]
        for q in range(4):
            mask=(fi>=q*run['expected_frames']/4)&(fi<(q+1)*run['expected_frames']/4)
            e=err[mask]
            quarters.append({'quarter':q+1,'associated_poses':int(mask.sum()),'ate_rmse_m':float(np.sqrt(np.mean(e**2))) if len(e) else None,'max_m':float(max(e)) if len(e) else None})
        output[run['mode']]={'same_global_alignment':True,'quarters':quarters,'source_run':run['run']}
        points.extend({'mode':run['mode'],'input_index':int(i),'timestamp':float(t),'error_m':float(e)} for i,t,e in zip(fi,est[0][ei],err))
    (ROOT/'euroc_accuracy_diagnostic.json').write_text(json.dumps(output,indent=2)+'\n')
    with (ROOT/'euroc_aligned_errors.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=['mode','input_index','timestamp','error_m'],lineterminator='\n');w.writeheader();w.writerows(points)
    print(json.dumps(output,indent=2))

if __name__=='__main__':main()
