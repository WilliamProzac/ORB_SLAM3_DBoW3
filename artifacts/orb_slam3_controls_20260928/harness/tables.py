#!/usr/bin/env python3
"""Export standalone figures and complete, explicitly scoped comparison tables."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from run import ROOT

def main():
    data=json.loads((ROOT/'analysis.json').read_text());lines=['# 单轮完整结果表','',
        '主表为各实现初始化后的全部输入，失败状态保留；完整区间及统一起始帧见 analysis.json。',
        'Track API 包含调用至返回的全部耗时，解码和外部诊断不计入。','']
    modes=['official800','delivered800','delivered700','gf_bridge']
    labels=['ORB3 official 800/8','ORB3 delivered 800/8','ORB3 delivered 700/5','GF03 baseline bridge']
    fig,axes=plt.subplots(1,2,figsize=(12,4.6),constrained_layout=True)
    for ax,dataset in zip(axes,['V1_01_easy','tum_large']):
        rr={r['mode']:r for r in data['runs'] if r['dataset']==dataset}
        if len(rr)!=4:continue
        lines+=['## '+dataset,'',f"统一比较起始输入索引：{rr['official800']['common_start_index']}。",'',
            '|版本|处理|non-OK|地图数|API mean / P95 / P99 / max ms|后25% P95 ms|共同区间 P95 ms|积压 P95 / 结束 ms|超周期帧|ATE m|',
            '|---|---:|---:|---:|---|---:|---:|---|---:|---:|']
        for mode in modes:
            r=rr[mode];t=r['after_initialization']['track_ms'];a=r.get('accuracy',{})
            timing=' / '.join(f'{t[k]:.3f}' for k in ['mean','p95','p99','max'])
            ate=f"{a['ate_rmse_m']:.4f}" if a else '无'
            if not r['accuracy_eligible']:ate+='（未过跟踪完整性门）'
            late=r['after_initialization']['input_lateness_ms']['p95']
            lines.append(f"|{mode}|{r['processed']}/{r['expected_frames']}|{r['non_ok_after_initialization']}|{r['max_maps']}|{timing}|{r['tail25']['track_ms']['p95']:.3f}|{r['common_range']['track_ms']['p95']:.3f}|{late:.3f} / {r['input_lateness_end_ms']:.3f}|{r['after_initialization']['over_period']}|{ate}|")
        lines+=['','|版本|ATE关联数|1秒 RPE 平移 / 旋转|RSS峰值 MiB|Swap峰值 MiB|平均进程CPU核数|最终KF / MP|',
            '|---|---:|---|---:|---:|---:|---|']
        for mode in modes:
            r=rr[mode];a=r.get('accuracy',{});res=r['resources']
            lines.append(f"|{mode}|{a.get('associated_poses',a.get('associated', '见JSON'))}|{a.get('rpe_translation_rmse_m',float('nan')):.4f} m / {a.get('rpe_rotation_rmse_deg',float('nan')):.4f}°|{res['rss_peak_kib']/1024:.1f}|{res['swap_peak_kib']/1024:.1f}|{res.get('mean_process_cores',0):.2f}|{r['final_kfs']} / {r['final_mps']}|")
        lines+=['']
        vals=[rr[m]['common_range']['track_ms']['p95'] for m in modes]
        bars=ax.barh(np.arange(4),vals,color=['#506784','#187c83','#62a4a0','#a2a2a2'])
        ax.set_yticks(np.arange(4));ax.set_yticklabels(labels);ax.invert_yaxis();ax.set_title(dataset)
        ax.axvline(rr['official800']['period_ms'],color='#ba623f',ls='--',lw=1,label='Input frame period')
        ax.set_xlabel('Track API P95 (ms), common frame range');ax.set_xlim(0,max(max(vals)*1.2,rr['official800']['period_ms']*1.14))
        if any(not rr[m]['tracking_pass'] for m in modes):ax.set_title(dataset+' (* tracking failure / split map)')
        for b,v,m in zip(bars,vals,modes):ax.text(v+.35,b.get_y()+b.get_height()/2,f'{v:.2f}'+(' *' if not rr[m]['tracking_pass'] else ''),va='center',fontsize=9)
        for spine in ['top','right']:ax.spines[spine].set_visible(False)
        ax.legend(loc='lower right',fontsize=8)
    (ROOT/'figures').mkdir(exist_ok=True)
    fig.savefig(ROOT/'figures/api_p95.png',dpi=180);fig.savefig(ROOT/'figures/api_p95.pdf');plt.close(fig)
    lines+=['## GF03 历史组合，以官方 ORB-SLAM3 为参照','',
        '下表 GF03 为上一批单轮数据；不是本轮重跑。百分比仅作历史数值参考，桥接漂移见报告。',
        '负值表示 P95 较低；跟踪/精度失败不能认定加速有效。','',
        '|数据|GF模式|历史P95 ms|官方P95 ms|数值变化|历史跟踪通过|历史ATE m|','|---|---|---:|---:|---:|---|---:|']
    for r in data['historical_GF03_reference']:
        lines.append(f"|{r['dataset']}|{r['mode']}|{r['track_p95_ms']:.3f}|{r['official_p95_ms']:.3f}|{r['change_vs_official_percent']:+.1f}%|{r['tracking_pass']}|{r['ate_rmse_m']:.4f}|")
    (ROOT/'DETAILS.md').write_text('\n'.join(lines)+'\n')
if __name__=='__main__':main()
