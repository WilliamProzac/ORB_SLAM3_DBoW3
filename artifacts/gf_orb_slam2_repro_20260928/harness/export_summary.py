#!/usr/bin/env python3
"""Compact evidence tables; performance ratios are not acceptance without tracking."""
import argparse
import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('matrices', nargs='+', type=Path)
    args = parser.parse_args()
    summary, rows = {}, []
    for matrix in args.matrices:
        assert not (matrix / 'INVALIDATED.json').exists(), 'Invalidated input experiment'
        evidence = json.loads((matrix / 'analysis.json').read_text())
        groups = evidence['groups']
        base = groups['baseline']
        comparisons = {}
        for mode, group in groups.items():
            if mode == 'baseline':
                continue
            comparisons[mode] = {
                'both_have_three_complete_loss_free_runs':
                    base['runs'] >= 3 and group['runs'] >= 3
                    and base['all_runs_tracking_pass'] and group['all_runs_tracking_pass'],
                'percent_change_candidate_vs_control': {
                    key: 100 * (group[key] / base[key] - 1)
                    for key in ['median_run_track_mean', 'median_run_track_p95',
                                'median_run_track_p99', 'median_run_tail_p95']
                },
                'interpretation': 'Negative timing change is faster; failed-run ratios are descriptive only.'
            }
        summary[matrix.name] = {'groups': groups, 'comparisons': comparisons}
        for run in evidence['runs']:
            timing = run.get('after_initialization', {}).get('track_ms', {})
            tail = run.get('tail25', {}).get('track_ms', {})
            accuracy = run.get('accuracy', {})
            resources = run.get('resources', {})
            rows.append(dict(matrix=matrix.name, repeat=run['repeat'], mode=run['mode'],
                             processed=run['processed'], expected=run['expected_frames'],
                             tracking_pass=run.get('tracking_pass', False),
                             non_ok=run.get('non_ok_after_initialization'),
                             mean_ms=timing.get('mean'), p95_ms=timing.get('p95'),
                             p99_ms=timing.get('p99'), max_ms=timing.get('max'),
                             tail_p95_ms=tail.get('p95'),
                             accuracy_eligible=run.get('accuracy_eligible', False),
                             ate_m=accuracy.get('ate_rmse_m'),
                             rpe_1s_translation_m=accuracy.get('rpe_translation_rmse_m'),
                             rpe_1s_rotation_deg=accuracy.get('rpe_rotation_rmse_deg'),
                             truth_associated_fraction=accuracy.get('associated_fraction_of_output'),
                             local_mp_mean=run.get('after_initialization', {}).get('local_mp', {}).get('mean'),
                             local_ms_mean=run.get('after_initialization', {}).get('local_ms', {}).get('mean'),
                             post_ms_mean=run.get('after_initialization', {}).get('post_ms', {}).get('mean'),
                             rss_peak_mib=resources.get('rss_peak_kib', 0)/1024,
                             swap_peak_kib=resources.get('swap_peak_kib'),
                             mean_process_cores=resources.get('mean_process_cores'),
                             source_run=run['run']))
    (ROOT / 'comparison_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    with (ROOT / 'run_summary.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f'Exported {len(rows)} rounds in {len(summary)} matrices')


if __name__ == '__main__':
    main()
