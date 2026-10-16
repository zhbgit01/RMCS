#!/usr/bin/env python3
"""Summarize complete 56 s yaw PID tests from ValueCollector CSV (angles in radians)."""
import argparse
import hashlib
import io
import json
import shutil
from pathlib import Path

import numpy as np


def analyze(csv_path, config, output):
    lines = csv_path.read_text().splitlines()
    expected_columns = len(lines[0].split(','))
    truncated_tail = bool(lines and len(lines[-1].split(',')) != expected_columns)
    if truncated_tail:
        lines.pop()  # A service stop may interrupt the final CSV write.
    data = np.genfromtxt(io.StringIO('\n'.join(lines)), delimiter=',', names=True,
                         deletechars='', encoding='utf-8')
    data = np.atleast_1d(data)
    def col(name):
        return data['/gimbal/yaw/' + name]
    time = col('pid_test/time')
    active = col('pid_test/active') > .5
    # Separate retries, including runs restarted between adjacent logged samples.
    starts = np.flatnonzero(active & np.r_[True, (~active[:-1]) | (np.diff(time) < 0)])
    summaries = []
    for run_id, start in enumerate(starts, 1):
        end = start + 1
        while end < len(data) and active[end] and time[end] >= time[end - 1]:
            end += 1
        t = time[start:end]
        err = np.rad2deg(col('control_angle_error')[start:end])
        torque = col('control_torque')[start:end]
        velocity = np.rad2deg(col('velocity_imu')[start:end])
        stage = col('pid_test/stage')[start:end]
        target = np.rad2deg(col('pid_test/offset')[start:end])
        finite = np.isfinite(np.column_stack([t, err, torque, velocity, target])).all()
        complete = bool(col('pid_test/done')[start:end].max() > .5)
        fault = bool(col('pid_test/fault')[max(start, end-1):min(len(data), end+2)].max() > .5)
        result = {'run': run_id, 'complete': complete, 'fault_near_exit': fault,
                  'finite': bool(finite), 'last_time_s': float(t[-1]),
                  'max_logged_interval_s': float(np.diff(t).max()) if len(t) > 1 else None,
                  'phases': {}, 'steps': []}
        if finite:
            for number, label in enumerate(['hold', 'steps', 'ramps', 'sine', 'final_hold']):
                mask = stage == number
                if not mask.any():
                    continue
                e, u, v = err[mask], torque[mask], velocity[mask]
                result['phases'][label] = {
                    'error_rmse_deg': float(np.sqrt(np.mean(e**2))),
                    'error_peak_deg': float(np.max(np.abs(e))),
                    'error_mean_deg': float(e.mean()),
                    'velocity_std_deg_s': float(v.std()),
                    'torque_peak_Nm': float(np.max(np.abs(u))),
                    'torque_rms_Nm': float(np.sqrt(np.mean(u**2))),
                    'torque_at_3Nm_fraction': float(np.mean(np.abs(u) >= 3 - 1e-6)),
                }
            # Use recorded target transitions: CSV timestamps are rounded, so a sample
            # printed as 11.000 may still belong to the preceding target plateau.
            indices = np.flatnonzero(stage == 1)
            boundaries = indices[np.r_[True, np.abs(np.diff(target[indices])) > 1]] if len(indices) else []
            for k, first in enumerate(boundaries):
                last = boundaries[k + 1] if k + 1 < len(boundaries) else indices[-1] + 1
                tt, ee = t[first:last], err[first:last]
                begin = float(tt[0])
                bad = np.flatnonzero(np.abs(ee) > .2)
                settled = 0 if not len(bad) else int(bad[-1] + 1)
                result['steps'].append({
                    'start_s': begin, 'target_offset_deg': float(target[first]),
                    'settling_time_within_0_2deg_s': float(tt[settled] - begin) if settled < len(tt) else None,
                    'last_0_5s_mean_error_deg': float(ee[tt >= tt[-1] - .5].mean()),
                })
        result['valid_for_comparison'] = bool(complete and finite and not fault
            and t[0] <= .02 and result['max_logged_interval_s'] is not None
            and result['max_logged_interval_s'] <= .02)
        summaries.append(result)
    output.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(config, output / 'config.yaml')
    report = {'source_csv': str(csv_path), 'discarded_truncated_final_row': truncated_tail, 'sha256': hashlib.sha256(csv_path.read_bytes()).hexdigest(),
              'config_note': 'Supplied config must match the deployed configuration at capture time.',
              'torque_limit_Nm_assumed': 3.0, 'runs': summaries}
    (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    if not any(r['valid_for_comparison'] for r in summaries):
        raise SystemExit('No complete, valid baseline: inspect faults, missing samples, or early cancellation.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--config', required=True, type=Path, help='Snapshot of deployed configuration')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    analyze(args.csv, args.config, args.output)
