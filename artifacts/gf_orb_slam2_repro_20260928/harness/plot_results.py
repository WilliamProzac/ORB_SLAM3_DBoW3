#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('matrix',type=Path);a=p.parse_args()
d=json.loads((a.matrix/'analysis.json').read_text())
modes=['baseline','mih32','mih_ots'];colors=['#586579','#db913e','#2b879b']
fig,axs=plt.subplots(2,2,figsize=(12,8))
for j,(mode,color) in enumerate(zip(modes,colors)):
    runs=[r for r in d['runs'] if r['mode']==mode and 'after_initialization' in r]
    for k,key in enumerate(['mean','p95']):
        v=[r['after_initialization']['track_ms'][key] for r in runs]
        axs[0,0].bar(j+(k-.5)*.34,np.median(v),width=.3,color=color,alpha=1 if k==0 else .5)
        axs[0,0].scatter([j+(k-.5)*.34]*len(v),v,c='black',s=12,zorder=5)
    if not runs:continue
    r=runs[0];f=Path(r['run'])/'frames.csv';x=np.genfromtxt(f,names=True,delimiter=',')
    width=80
    xx=[np.mean(x['index'][i:i+width]) for i in range(0,len(x),width)]
    for ax,col in [(axs[0,1],'track_ms'),(axs[1,0],'local_mp')]:
        yy=[np.median(x[col][i:i+width]) for i in range(0,len(x),width)]
        ax.plot(xx,yy,color=color,label=mode,linewidth=1.7)
    yy=[q['accuracy']['ate_rmse_m']*100 for q in runs if q.get('accuracy_eligible')]
    if yy:axs[1,1].scatter([j]*len(yy),yy,color=color,s=48)
    failed=sum(not q.get('tracking_pass') for q in runs)
    axs[1,1].text(j,.97,f'{len(yy)}/{len(runs)} eligible\n{failed} tracking failures',ha='center',va='top',transform=axs[1,1].get_xaxis_transform(),fontsize=8)
axs[0,0].set_xticks(range(3));axs[0,0].set_xticklabels(modes)
axs[0,0].set_title('Complete Track call: mean (solid) / P95 (faded)')
axs[0,0].set_ylabel('milliseconds; median across repeats, dots = runs')
axs[0,1].set_title('First repeat: Track latency, 80-frame median')
axs[0,1].set_ylabel('milliseconds');axs[0,1].set_xlabel('source frame index');axs[0,1].legend()
axs[1,0].set_title('First repeat: local map entering search')
axs[1,0].set_ylabel('map points');axs[1,0].set_xlabel('source frame index');axs[1,0].legend()
axs[1,1].set_title('Online ATE, only complete runs without track loss')
axs[1,1].set_ylabel('centimetres');axs[1,1].set_xticks(range(3));axs[1,1].set_xticklabels(modes);axs[1,1].set_xlim(-.5,2.5)
for ax in axs.ravel():ax.grid(alpha=.2,axis='y');ax.set_axisbelow(True)
fig.suptitle(a.matrix.name+' | 800 features, CPU, no loop closure; native API, not ROS callback',fontsize=12)
fig.tight_layout(rect=[0,0,1,.96]);fig.savefig(a.matrix/'comparison.png',dpi=170);fig.savefig(a.matrix/'comparison.pdf')
