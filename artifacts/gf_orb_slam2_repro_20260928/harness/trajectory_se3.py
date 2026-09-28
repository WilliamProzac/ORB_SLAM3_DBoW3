#!/usr/bin/env python3
"""SE(3)-only ATE and fixed 1-second RPE for timestamped TUM camera poses."""
import argparse
import bisect
import json
from pathlib import Path
import numpy as np


def read_poses(path):
    rows = np.loadtxt(path, comments='#', ndmin=2)
    if rows.shape[1] != 8 or len(rows) < 3 or not np.isfinite(rows).all():
        raise ValueError('expected at least three finite TUM poses')
    if np.any(np.diff(rows[:, 0]) <= 0): raise ValueError('timestamps must increase')
    q = rows[:, 4:8]
    norms = np.linalg.norm(q, axis=1)
    if np.any(norms < 1e-8): raise ValueError('invalid quaternion')
    x, y, z, w = (q / norms[:, None]).T
    rotations = np.empty((len(rows), 3, 3))
    rotations[:, 0, 0] = 1-2*(y*y+z*z)
    rotations[:, 0, 1] = 2*(x*y-z*w)
    rotations[:, 0, 2] = 2*(x*z+y*w)
    rotations[:, 1, 0] = 2*(x*y+z*w)
    rotations[:, 1, 1] = 1-2*(x*x+z*z)
    rotations[:, 1, 2] = 2*(y*z-x*w)
    rotations[:, 2, 0] = 2*(x*z-y*w)
    rotations[:, 2, 1] = 2*(y*z+x*w)
    rotations[:, 2, 2] = 1-2*(x*x+y*y)
    return rows[:, 0], rows[:, 1:4], rotations


def associate(a, b, tolerance):
    candidates = []
    for i, stamp in enumerate(a):
        lo, hi = bisect.bisect_left(b, stamp-tolerance), bisect.bisect_right(b, stamp+tolerance)
        for j in range(lo, hi): candidates.append((abs(stamp-b[j]), i, j))
    used_a, used_b, pairs = set(), set(), []
    for _, i, j in sorted(candidates):
        if i not in used_a and j not in used_b:
            pairs.append((i, j)); used_a.add(i); used_b.add(j)
    return sorted(pairs)


def evaluate(estimated, truth, tolerance=.02, interval=1., interval_tolerance=.02):
    et, ep, er = estimated
    gt, gp, gr = truth
    pairs = associate(et, gt, tolerance)
    if len(pairs) < 3: raise ValueError('insufficient timestamp associations')
    ei, gi = np.array(pairs).T
    ep, er, et = ep[ei], er[ei], et[ei]
    gp, gr = gp[gi], gr[gi]
    ec, gc = ep.mean(axis=0), gp.mean(axis=0)
    u, _, vt = np.linalg.svd((ep-ec).T @ (gp-gc))
    sign = np.eye(3); sign[2, 2] = np.linalg.det(vt.T @ u.T)
    alignment = vt.T @ sign @ u.T
    translation = gc-alignment @ ec
    ate = np.linalg.norm(ep @ alignment.T + translation-gp, axis=1)
    rpe_t, rpe_r, durations = [], [], []
    for i, stamp in enumerate(et):
        k = int(np.searchsorted(et, stamp+interval))
        candidates = [j for j in (k-1, k) if i < j < len(et)]
        if not candidates: continue
        j = min(candidates, key=lambda j: abs(et[j]-stamp-interval))
        if abs(et[j]-stamp-interval) > interval_tolerance: continue
        delta_er, delta_gr = er[i].T @ er[j], gr[i].T @ gr[j]
        delta_ep, delta_gp = er[i].T @ (ep[j]-ep[i]), gr[i].T @ (gp[j]-gp[i])
        error_r = delta_gr.T @ delta_er
        error_t = delta_gr.T @ (delta_ep-delta_gp)
        rpe_t.append(np.linalg.norm(error_t))
        rpe_r.append(np.degrees(np.arccos(np.clip((np.trace(error_r)-1)/2, -1, 1))))
        durations.append(float(et[j]-stamp))
    if not rpe_t: raise ValueError('no fixed-interval RPE pairs')
    def rms(values): return float(np.sqrt(np.mean(np.square(values))))
    return {'estimated_poses': len(estimated[0]), 'associated_poses': len(pairs),
            'timestamp_tolerance_s': tolerance, 'alignment': 'SE3 (scale fixed at 1)',
            'alignment_rotation': alignment.tolist(), 'alignment_translation': translation.tolist(),
            'ate_rmse_m': rms(ate), 'ate_median_m': float(np.median(ate)),
            'rpe_interval_s': interval, 'rpe_interval_tolerance_s': interval_tolerance,
            'rpe_pairs': len(rpe_t), 'rpe_translation_rmse_m': rms(rpe_t),
            'rpe_rotation_rmse_deg': rms(rpe_r),
            'rpe_actual_interval_range_s': [min(durations), max(durations)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('estimated', type=Path)
    parser.add_argument('groundtruth', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(read_poses(args.estimated), read_poses(args.groundtruth))
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__': main()
