#!/usr/bin/env python3
"""Compare measured pre-postprocessing time with whole Track API time."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('matrix',type=Path);a=p.parse_args()
d=json.loads((a.matrix/'analysis.json').read_text());modes=['baseline','mih_ots','paper_ots','paper_all']
fig,axs=plt.subplots(1,2,figsize=(12,4.8))
for ax,stat in zip(axs,['mean','p95']):
 for k,(field,label,color) in enumerate([('pose_ready_ms','Pose-ready boundary','#398c99'),('track_ms','Whole Track API','#d19a4d')]):
  values=[[r['after_initialization'][field][stat] for r in d['runs'] if r['mode']==m] for m in modes]
  positions=np.arange(4)+(k-.5)*.34
  ax.bar(positions,[np.median(v) for v in values],width=.31,color=color,label=label)
  for x,v in zip(positions,values):ax.scatter([x]*len(v),v,c='black',s=13,zorder=3)
 ax.set_title(stat.upper()+' across initialized frames')
 ax.set_xticks(range(4));ax.set_xticklabels(modes);ax.set_ylabel('milliseconds');ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
axs[0].legend()
fig.suptitle(a.matrix.name+' | median across 3 repeats; dots = runs\nPose-ready is before postprocessing, not ROS publication; failed runs retained',fontsize=11,y=.98)
fig.tight_layout(rect=[0,0,1,.86]);fig.savefig(a.matrix/'timing_boundary.png',dpi=170);fig.savefig(a.matrix/'timing_boundary.pdf')
