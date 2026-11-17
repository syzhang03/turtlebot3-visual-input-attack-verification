#!/usr/bin/env python3
"""Audit completed campaign evidence and create a bounded stability report."""
import argparse
import json
from pathlib import Path

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('folder')
args=parser.parse_args()
folder=Path(args.folder)
summary=json.loads((folder/'summary.json').read_text())
protocol=json.loads((folder/'protocol.json').read_text())
runs=json.loads((folder/'runs.json').read_text())
groups=summary['groups']
expected=7*protocol['repeats']+3*protocol['fault_repeats']
checks={'all_attempts_retained':len(runs)==expected,
        'all_predeclared_trial_outcomes':all(r['passed'] for r in runs),
        'frozen_source_and_parameters':summary['source_unchanged'],
        'final_speed_below_0.02_m_s':all(abs(r.get('final_speed_m_s',999))<.02 for r in runs),
        'minimum_pose_bound_0.35_m':all(r.get('min_pose_clearance_bound_m') is None
                                      or r['min_pose_clearance_bound_m']>=.35 for r in runs),
        'normal_inputs_no_anomaly_alerts':not any(r.get('anomaly_alert') for r in runs
                   if r['condition']=='obstacle-clean' or r['condition'].startswith('empty-')),
        'one_second_and_mask_detected':all(r.get('anomaly_alert') for r in runs
                   if r['condition'] in ('obstacle-delay-1','obstacle-occlusion')),
        'all_empty_tasks_reached_goal':all(r.get('state')=='GOAL_REACHED' for r in runs
                   if r['condition'].startswith('empty-')),
        'all_faults_stopped_on_timeout':all(r.get('state')=='SENSOR_TIMEOUT' for r in runs
                   if r['condition'] in ('camera-loss','scan-loss','camera-freeze')),
        'fault_injection_recorded':all(r.get('fault_activated_sim_s') is not None for r in runs
                   if r['condition'] in ('camera-loss','scan-loss','camera-freeze'))}
audit=dict(checks=checks,stable_within_tested_scope=all(checks.values()),
           note='Repeatability in these fixed conditions, not proof of general reliability or fusion superiority.')
(folder/'stability-audit.json').write_text(json.dumps(audit,indent=2)+'\n')
lines=['# Frozen Camera LiDAR validation campaign','',
       f"Completed {len(runs)}/{expected} predeclared attempts. Frozen-source check: {summary['source_unchanged']}.",'',
       '| Condition | Expected outcome | Anomaly alerts | Min pose clearance bound (m) | Mean stop time (sim s) |',
       '|---|---:|---:|---:|---:|']
for name,g in groups.items():
    gap=g['ground_truth_gap_m']['minimum']; duration=g['stop_time_sim_s']['mean']
    gap_text=f'{gap:.3f}' if gap is not None else 'N/A (empty world)'
    time_text=f'{duration:.3f}' if duration is not None else 'N/A'
    lines.append(f"| {name} | {g['passed']}/{g['total']} | {g['alerts']}/{g['total']} | {gap_text} | {time_text} |")
lines += ['', '## Stability audit','']
lines += [f"- {'PASS' if passed else 'FAIL'}: {name}" for name,passed in checks.items()]
lines += ['', '## Interpretation','',
          'The 0.5 s replay condition permits either anomaly stopping or normal LiDAR stopping. Its alert count is reported separately from safe-stop success; missed anomaly confirmation is not hidden by the safety metric.',
          'Normal and empty-task alarm counts are counts over this finite campaign, not a general false-positive probability. Empty attacked routes intentionally contain no obstacle, so camera masking need not imply cross-sensor disagreement.',
          'Fault injection occurs only after RUNNING. Camera freeze repeats the last complete image and its timestamp; it is distinct from the replay attack that rewrites current headers.',
          'Clearance is a conservative bound from sampled native simulator poses and a verified robot enclosure. It is independent of LiDAR/odometry but not a contact sensor or exact mesh distance.',
          'No noise is added and no explicit random seed is configured. Natural execution jitter is not a controlled noise study. A separate multi-controller evaluation is needed to assess fusion benefit.',
          'This report does not approve or merge a PR. Independent teammate review remains separate from agent-run experiments.','']
(folder/'REPORT.md').write_text('\n'.join(lines))
print(json.dumps(audit,indent=2))
