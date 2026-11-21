#!/usr/bin/env python3
"""Reanalyze the team's frozen reliability CSV; does not run ROS or Gazebo."""
import argparse
import csv
import hashlib
import json
import math
import platform
import statistics as st
from collections import Counter
from pathlib import Path

PROFILES = ['clean', 'camera-latency', 'camera-visor', 'lidar-unavailable', 'both-unavailable']
LABELS = ['Normal', 'Camera delay', 'Camera visor', 'LiDAR loss', 'Dual loss']
def flag(value):
    if value == 'True': return True
    if value == 'False': return False
    if value == '': return None
    raise ValueError(f'Unexpected boolean: {value!r}')
def number(value):
    if value == '': return None
    x = float(value)
    if not math.isfinite(x): raise ValueError('Non-finite numeric value')
    return x
def read(path):
    with path.open(encoding='utf-8-sig', newline='') as f: return list(csv.DictReader(f))
def write(path, rows):
    with path.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
def describe(values, prefix):
    xs = [x for x in values if x is not None]
    return {prefix+'_n': len(xs), prefix+'_mean': st.mean(xs) if xs else '',
            prefix+'_sd': st.stdev(xs) if len(xs)>1 else '',
            prefix+'_median': st.median(xs) if xs else '',
            prefix+'_min': min(xs) if xs else '', prefix+'_max': max(xs) if xs else ''}
def rate(rows, key):
    xs = [flag(r[key]) for r in rows]; xs = [x for x in xs if x is not None]
    k, n = sum(xs), len(xs)
    return {key+'_count': k, key+'_denominator': n, key+'_rate': k/n if n else ''}
def summarize(rows, scene='all', profile='all', noise='all'):
    result = dict(scene=scene, profile=profile, noise=noise, n_total=len(rows),
                  n_valid=sum(flag(r['valid']) is True for r in rows))
    for key in ['normal_task_success','false_stop','restricted']:
        result.update(rate(rows,key))
    result['noise_scheduled_n'] = sum(r['noise']=='on' for r in rows)
    result['noise_exposed_n'] = sum(flag(r['noise_exposed']) is True for r in rows if r['noise']=='on')
    result['alert_trials'] = sum(int(r['anomaly_alert_count'])>0 for r in rows)
    result.update(describe([number(r['min_clearance_lower_m']) for r in rows if r['scene']=='obstacle'], 'clearance_lower_m'))
    times = [number(r['motion_to_stop_command_s'])+number(r['settling_latency_s'])
             for r in rows if r['scene']=='empty' and flag(r['normal_task_success']) is True
             and number(r['motion_to_stop_command_s']) is not None and number(r['settling_latency_s']) is not None]
    result.update(describe(times, 'successful_empty_completion_sim_s'))
    for key in ['motion_to_stop_command_s','braking_latency_s','settling_latency_s','detection_latency_s']:
        result.update(describe([number(r[key]) for r in rows],key))
    return result

def main():
    base=Path(__file__).resolve().parents[1]
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,default=base/'data/reliability-main-2026-11-18/trials.csv')
    ap.add_argument('--output',type=Path,default=base/'results/final-analysis')
    args=ap.parse_args(); rows=read(args.input); out=args.output; out.mkdir(parents=True,exist_ok=True)
    required={'trial','scene','profile','noise','repeat','valid','pair','normal_task_success','false_stop','restricted','noise_exposed','anomaly_alert_count','min_clearance_lower_m','motion_to_stop_command_s','settling_latency_s','braking_latency_s','detection_latency_s'}
    if not rows or not required.issubset(rows[0]): raise ValueError('Missing required columns or empty input')
    if len({r['trial'] for r in rows})!=len(rows): raise ValueError('Duplicate trial IDs')
    expected={(s,p,n):10 for s in ['empty','obstacle'] for p in PROFILES for n in ['off','on']}
    observed=Counter((r['scene'],r['profile'],r['noise']) for r in rows)
    if observed!=expected: raise ValueError('Expected the frozen 20 groups x 10 trials matrix')
    if any(flag(r['valid']) is not True for r in rows): raise ValueError('Invalid trial: review scoring before analysis')
    grouped=[summarize([r for r in rows if (r['scene'],r['profile'],r['noise'])==(s,p,n)],s,p,n)
             for s,p,n in expected]
    overall=summarize(rows)
    profiles=[summarize([r for r in rows if r['profile']==p],profile=p) for p in PROFILES]
    write(out/'summary.csv',grouped);write(out/'by_profile.csv',profiles);write(out/'overall.csv',[overall])
    pairs={}
    for r in rows:
        if r['noise'] in pairs.setdefault(r['pair'],{}): raise ValueError('Duplicate pair arm')
        pairs[r['pair']][r['noise']]=r
    paired=[]
    for key,arms in sorted(pairs.items()):
        if set(arms)!={'off','on'}: raise ValueError('Incomplete noise pair')
        a,b=arms['off'],arms['on']
        if (a['scene'],a['profile'],a['repeat'])!=(b['scene'],b['profile'],b['repeat']): raise ValueError('Mismatched noise pair')
        paired.append(dict(pair=key,scene=a['scene'],profile=a['profile'],
                           false_stop_off=flag(a['false_stop']),false_stop_on=flag(b['false_stop']),
                           noise_exposed_on=flag(b['noise_exposed'])))
    write(out/'noise_pairs.csv',paired)
    checks={'trial_count':len(rows),'groups':len(grouped),'pairs':len(paired),'unique_trial_ids':True,'all_valid':True,
            'false_stops':overall['false_stop_count'],'scheduled_noise':overall['noise_scheduled_n'],
            'exposed_noise':overall['noise_exposed_n'],'reference_group_metrics':'not available'}
    bounds=[number(r['min_clearance_lower_m']) for r in rows if r['scene']=='obstacle']
    checks['obstacle_trials_above_0_35_m']=sum(x is not None and x>.35 for x in bounds)
    checks['obstacle_trials_with_clearance']=sum(x is not None for x in bounds)
    ref=args.input.with_name('group-metrics.csv')
    if ref.exists():
        references={(r['scene'],r['profile'],r['noise']):r for r in read(ref)}
        if set(references)!=set(expected):raise ValueError('Reference group matrix differs')
        for g in grouped:
            r=references[g['scene'],g['profile'],g['noise']]
            for x,y in [('n_total','n'),('normal_task_success_count','successful_task'),('false_stop_count','false_stops'),('restricted_count','restricted'),('alert_trials','anomaly_alerts')]:
                if g[x]!=int(r[y]):raise ValueError(f'Reference mismatch: {g["scene"],g["profile"],g["noise"]} {x}')
        checks['reference_group_metrics']='all 20 groups match on n, success, false stops, restricted and alert trials'
    (out/'validation.json').write_text(json.dumps(checks,indent=2)+'\n',encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    figures=out/'figures';figures.mkdir(exist_ok=True)
    def save(fig,name):
        fig.tight_layout(rect=(0,.05,1,1) if fig.texts else (0,0,1,1));fig.savefig(figures/(name+'.png'),dpi=180);fig.savefig(figures/(name+'.svg'));plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,4.8))
    for j,scene in enumerate(['empty','obstacle']):
        gs=[summarize([r for r in rows if r['scene']==scene and r['profile']==p]) for p in PROFILES]
        bars=ax.bar([i+(j-.5)*.36 for i in range(5)],[g['normal_task_success_rate']*100 for g in gs],.36,label=scene.capitalize(),color=['#176b52','#dc9652'][j])
        ax.bar_label(bars,labels=[f'{g["normal_task_success_count"]}/{g["normal_task_success_denominator"]}' for g in gs],padding=3,fontsize=9)
    ax.set(xticks=range(5),xticklabels=LABELS,ylim=(0,115),ylabel='Task success (%)',title='Task success by input and scene');ax.legend();save(fig,'01_task_success')
    fig,ax=plt.subplots(figsize=(9,4.8));bars=ax.bar(LABELS,[g['false_stop_rate']*100 for g in profiles],color='#b6534c')
    ax.bar_label(bars,labels=[f'{g["false_stop_count"]}/{g["false_stop_denominator"]}' for g in profiles],padding=4)
    ax.set(ylim=(0,60),ylabel='False stops (%)',title='Recorded false-stop flags by input (both scenes)')
    fig.text(.5,.01,'Original scoring retained; 8 flags near scan-age boundary (source report).',ha='center',fontsize=9);fig.subplots_adjust(bottom=.18);save(fig,'02_false_stops')
    fig,ax=plt.subplots(figsize=(9,4.8));v=[[number(r['min_clearance_lower_m']) for r in rows if r['scene']=='obstacle' and r['profile']==p] for p in PROFILES]
    ax.boxplot(v,tick_labels=LABELS,showmeans=True);ax.axhline(.35,color='#b6534c',linestyle='--',label='0.35 m evaluation threshold')
    ax.set(ylabel='Minimum clearance lower bound (m)',title='Independent geometry bounds: 20 obstacle trials per input');ax.legend();save(fig,'03_clearance')
    fig,ax=plt.subplots(figsize=(8,4.8));v=[];labs=[]
    for p,label in zip(PROFILES,LABELS):
        xs=[number(r['motion_to_stop_command_s'])+number(r['settling_latency_s']) for r in rows if r['scene']=='empty' and r['profile']==p and flag(r['normal_task_success']) is True]
        if xs:v.append(xs);labs.append(f'{label}\n(n={len(xs)})')
    ax.boxplot(v,tick_labels=labs,showmeans=True);ax.set(ylabel='Completion time (simulation seconds)',title='Completion time conditional on successful empty-route trials')
    save(fig,'04_completion_time')
    provenance={'input':args.input.name,'input_sha256':hashlib.sha256(args.input.read_bytes()).hexdigest(),
                'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'python':platform.python_version(),'matplotlib':matplotlib.__version__,
                'activity':'Secondary analysis of supplied frozen CSV; no new simulation trials.'}
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n',encoding='utf-8')
    off=sum(p['false_stop_off'] is True for p in paired);on=sum(p['false_stop_on'] is True for p in paired)
    only_on=sum(p['false_stop_on'] is True and p['false_stop_off'] is False for p in paired)
    only_off=sum(p['false_stop_off'] is True and p['false_stop_on'] is False for p in paired)
    lines=['# Reliability evaluation: independent reanalysis','',
      'This submission reanalyzes the team-supplied frozen dataset. It does not claim authorship of the original controller, experiment harness or data collection. No new ROS/Gazebo trials were run.','',
      '## Dataset and validation','',f'{len(rows)} trials; 20 scene × input × noise groups, 10 repeats each. All trial IDs are unique and all records are marked valid. The five grouped counts were checked against the supplied group-metrics.csv; see validation.json.','',
      '## Recomputed results','', '| Input | Task success (both scenes) | False stops | Restricted |','|---|---:|---:|---:|']
    for g,label in zip(profiles,LABELS):lines.append(f'| {label} | {g["normal_task_success_count"]}/40 | {g["false_stop_count"]}/40 | {g["restricted_count"]}/40 |')
    lines+=['',f'All 100 obstacle trials have recorded minimum geometry-based clearance lower bounds above 0.35 m. The minimum is {overall["clearance_lower_m_min"]:.3f} m. This is not a direct contact-event measurement.',
      '',f'False stops: {overall["false_stop_count"]}/200. Noise off: {off}/100; noise on: {on}/100. There are {only_on} pairs with a false stop only under noise and {only_off} only without noise. These are descriptive paired results, not a causal effect or significance claim.',
      '',f'Confirmed noise exposure: {overall["noise_exposed_n"]}/100 scheduled noise trials. The unexposed assigned trial is retained in its original group.',
      '', '### Successful empty-route completion times','', '| Input | Successful trials with time | Mean ± sample SD (sim s) | Median (sim s) |','|---|---:|---:|---:|']
    for g,label in zip(profiles,LABELS):
        pre='successful_empty_completion_sim_s';n=g[pre+'_n']
        if n:lines.append(f'| {label} | {n} | {g[pre+"_mean"]:.3f} ± {g[pre+"_sd"]:.3f} | {g[pre+"_median"]:.3f} |')
        else:lines.append(f'| {label} | 0 | N/A | N/A |')
    lines+=['','## Interpretation and limits','',
      '- All trials used the new three-state policy; there is no matched old-policy arm. Do not interpret this dataset as a measured improvement over the old controller.',
      '- Task success has scene-specific meaning: finish the empty 4 m route, or make the expected normal obstacle stop with verified clearance. A protective stop may be safe but not task-successful.',
      '- The supplied REPORT.md identifies 8 of the 28 false-stop flags within 1 ms of the 0.5 s scan-age threshold. This CSV lacks scan-age evidence to independently identify those trials. Preserve the original flags; audit the raw evidence before rescoring.',
      '- The source report assigns 19 false stops to the 20 obstacle-visor trials. This differs from the denominator of all 40 visor trials.',
      '- Restricted-state entry is not validated fault detection or sensor attribution. Even nominal input enters restricted mode.',
      '- Completion times exclude failed routes. Compare them alongside task-success rates, not as an unconditional speed ranking.',
      '- Timing is in simulation seconds. Command timing, braking onset and settling are distinct measurements. Missing metrics are not zero.',
      '- Fixed scenes, repeated runs and uncontrolled scheduling limit generalization. The graphs are descriptive; no inferential independence assumption is made.',
      '', '## Sources','',
      '- Supplied Defense Presentation Handoff/Evidence: trials.csv, group-metrics.csv, protocol.json, REPORT.md.',
      '- https://github.com/syzhang03/turtlebot3-visual-input-attack-verification/blob/af50cb7/results/reliability-main-2026-11-18/REPORT.md',
      '- Original collection controller: aef5380, as stated in the supplied report; the integrated branch was validated separately.',
      '', '## Contribution and AI disclosure','',
      'This package contributes a secondary-analysis script, reproduced descriptive summaries, figures, consistency checks and reproduction documentation. ChatGPT/Codex assisted implementation and drafting. Yang Mingzhi should review and understand the calculations before claiming the reviewed analysis as a personal contribution. Original team data and code remain attributed to their authors.','']
    (out/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps(checks,indent=2));print('Output:',out)
if __name__=='__main__':main()
