"""Summarize matched runs with explicit missing measurements and denominators."""
import argparse
import json
from pathlib import Path
from statistics import mean, stdev


def describe(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return dict(n=len(values), mean=mean(values), sd=stdev(values) if len(values)>1 else None,
                minimum=min(values), maximum=max(values))


def outcome_rate(rows, metric):
    known = [r[metric] for r in rows if r.get(metric) is not None]
    return dict(positive=sum(known), total=len(rows), unknown=len(rows)-len(known))


def format_rate(rate):
    if rate is None:
        return 'N/A'
    unknown = f" ({rate['unknown']} unknown)" if rate['unknown'] else ''
    return f"{rate['positive']}/{rate['total']}{unknown}"


def format_mean(stats, count):
    if stats is None:
        return 'N/A'
    return f"{stats['mean']:.3f} ({stats['n']}/{count} measured)"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder')
    args = parser.parse_args()
    folder = Path(args.folder)
    protocol = json.loads((folder/'protocol.json').read_text())
    audited = (folder/'analysis-status.json').exists()
    rows = json.loads((folder/('analysis-runs.json' if audited else 'runs.json')).read_text())
    status = json.loads((folder/('analysis-status.json' if audited else 'status.json')).read_text())
    if protocol['pilot'] or status['completed'] != status['planned']:
        raise ValueError('Use a completed formal batch')
    statistics = []
    for group, controller, attack, empty in protocol['plan']:
        sample = [r for r in rows if (r['group'], r['controller'], r['input'], r['empty']) ==
                  (group, controller, attack, empty)]
        statistics.append(dict(group=group, input=attack, scene='empty' if empty else 'obstacle',
            planned=len(sample), valid=sum(r['valid'] for r in sample),
            attack_success=outcome_rate(sample,'attack_success') if attack!='clean' else None,
            normal_task_success=outcome_rate(sample,'normal_task_success') if attack=='clean' else None,
            false_alarm=outcome_rate(sample,'false_alarm') if attack=='clean' and controller=='fusion' else None,
            premature_stop=outcome_rate(sample,'premature_stop') if empty else None,
            alerts=sum(r.get('first_alert_sim_s') is not None for r in sample),
            missing_detection_reasons=[r.get('detection_note',r.get('error')) for r in sample
                                       if attack!='clean' and r.get('detection_latency_s') is None],
            missing_braking_reasons=[r.get('braking_note',r.get('error')) for r in sample if r.get('braking_latency_s') is None],
            **{key:describe([r.get(key) for r in sample]) for key in
               ['min_clearance_lower_m','min_clearance_upper_m','detection_latency_s',
                'braking_latency_s','settling_latency_s','motion_to_stop_command_s']}))
    (folder/'metrics.json').write_text(json.dumps(dict(status=status,conditions=statistics),indent=2)+'\n')
    lines = ['# Issue 13 four-group evaluation', '',
             f"Recorded {status['completed']} of {status['planned']} planned attempts. "
             f"{status['valid']} yielded analyzable evidence. Frozen source check: {status['source_unchanged']}.", '',
             '| Group / input / scene | Attack success | Normal task success | False alarms | Premature stops |',
             '|---|---:|---:|---:|---:|']
    for row in statistics:
        label = f"{row['group']} / {row['input']} / {row['scene']}"
        values = [format_rate(row[key]) for key in ('attack_success','normal_task_success','false_alarm','premature_stop')]
        lines.append('| '+' | '.join([label,*values])+' |')
    lines += ['', '| Group / input / scene | Clearance bound range across runs (m) | Mean detection (sim s) | Mean braking onset (sim s) | Mean settling (sim s) |',
              '|---|---:|---:|---:|---:|']
    for row in statistics:
        lo, hi = row['min_clearance_lower_m'], row['min_clearance_upper_m']
        gap = f"[{lo['minimum']:.3f}, {hi['maximum']:.3f}]" if lo and hi else 'N/A'
        values = [format_mean(row[key],row['planned']) for key in ('detection_latency_s','braking_latency_s','settling_latency_s')]
        lines.append('| '+' | '.join([f"{row['group']} / {row['input']} / {row['scene']}",gap,*values])+' |')
    lines += ['', 'See `metrics.json` for separate lower/upper-bound statistics and per-condition timing ranges. '
              'A clearance interval reflects uncertainty in sampled collision geometry. A threshold violation is '
              'confirmed only when the upper bound is below 0.35 m. The lower bound establishes a safe margin '
              'when it reaches 0.35 m. The contact event rate is unavailable because these runs have no contact sensor.', '',
              'Detection uses the first accepted manipulated frame and the confirmed alert on simulation time. '
              'One occlusion trial accepted its first manipulated frame before its controller clock synchronized; its detection latency is unavailable. '
              'Braking onset uses sustained independently measured speed decrease after the stop command. '
              'Trials where motion slowed before the command or no stop command occurred have unavailable braking latency. '
              'See `analysis-runs.json` (or `runs.json` before audit) for those reasons and any indeterminate outcomes.', '']
    (folder/'REPORT.md').write_text('\n'.join(lines))
    print(json.dumps(status))


if __name__ == '__main__':
    main()
