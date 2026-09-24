#!/usr/bin/env python3
"""Generate compact JSON/CSV from preserved D435i joint-run evidence."""
import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    rows = []
    for path in sorted(args.root.glob('*/result.json')):
        review = path.with_name('analysis_review.json')
        result = json.loads((review if review.exists() else path).read_text())
        bench = result.get('benchmark', {})
        hist = bench.get('callback_ms', {})
        memory = result.get('memory', {})
        rows.append(dict(run=path.parent.name, timing=result['timing_mode'],
            completed=result['completed'], static_integration_ok=result.get('static_integration_ok', False),
            input_adapter_ok=result.get('input_adapter_ok', False),
            production_realtime_passed=result.get('production_realtime_passed', False),
            input_rgb=result.get('rgb', {}).get('messages'),
            input_depth=result.get('depth', {}).get('messages'),
            missing_processed=len(result.get('rgb', {}).get('missing_processed', [])),
            callbacks=bench.get('counts', [None, None, None])[2], non_ok=bench.get('non_ok'),
            keyframes=bench.get('keyframes'), map_points=bench.get('map_points'),
            callback_p50_ms=hist.get('p50'), callback_p95_ms=hist.get('p95'),
            callback_p99_ms=hist.get('p99'), callback_max_ms=bench.get('exact_max_ms'),
            peak_rss_mib=memory.get('rss_kib', {}).get('max', 0)/1024,
            peak_pss_mib=memory.get('pss_kib', {}).get('max', 0)/1024,
            peak_swap_kib=memory.get('swap_kib', {}).get('max'),
            min_available_mib=memory.get('available_kib', {}).get('min', 0)/1024,
            error=result.get('error', result.get('analysis_error', result.get('original_analysis_error') or ''))))
    (args.root/'SUMMARY.json').write_text(json.dumps(rows, indent=2, ensure_ascii=False)+'\n')
    if rows:
        with (args.root/'SUMMARY.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    print(json.dumps(rows, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
