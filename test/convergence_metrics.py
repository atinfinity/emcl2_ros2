#!/usr/bin/env python3
# Copyright 2026 emcl2_ros2 developers
# SPDX-FileCopyrightText: 2026 emcl2_ros2 developers
# SPDX-License-Identifier: LGPL-3.0-or-later
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
Convergence metrics for the recovery / global-localization eval scenario.

The tracking eval reports APE over the whole run, but for a recovery scenario
the estimate starts far from the truth (uniform belief or a wrong initial
pose) and the interesting question is *whether and how fast* it converges to
the true pose -- an APE average would be dominated by the pre-convergence
phase. This script associates the estimate with ground truth by nearest
timestamp and reports:
  converged            : did the translation error settle below the threshold
                         and stay there until the end of the run?
  time_to_converge_s   : seconds from the first sample until it did
  post_converge_rmse_m : RMSE of the translation error after convergence
  final_error_m        : translation error of the last associated sample
  max_error_m          : worst translation error over the run

Usage: convergence_metrics.py <ground_truth.tum> <mcl_pose.tum> [threshold_m]
Prints a human-readable block to stderr and a single key=value line (the
machine-readable summary) to stdout.
"""

import sys


def load(path):
    rows = []
    try:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                parts = line.split()
                try:
                    t, x, y = float(parts[0]), float(parts[1]), float(parts[2])
                except (ValueError, IndexError):
                    # skip a malformed line (e.g. a partially flushed final row)
                    continue
                rows.append((t, x, y))
    except OSError:
        return []
    return rows


def associate(gt, est, max_diff=0.1):
    """Match each estimate sample to the nearest-in-time ground-truth sample."""
    pairs = []
    j = 0
    for te, xe, ye in est:
        # advance the gt cursor to the closest timestamp
        while j + 1 < len(gt) and abs(gt[j + 1][0] - te) <= abs(gt[j][0] - te):
            j += 1
        tg, xg, yg = gt[j]
        if abs(tg - te) <= max_diff:
            err = ((xe - xg) ** 2 + (ye - yg) ** 2) ** 0.5
            pairs.append((te, err))
    return pairs


def detect_kidnap(gt, min_jump=0.5):
    """Return the time of the largest ground-truth jump if it exceeds min_jump."""
    # Normal driving never moves that far between samples, so a large single-step
    # jump marks a deliberate mid-run teleport (the kidnapped-robot scenario).
    # Returns None when no such jump is present (the ordinary recovery scenarios).
    kidnap_t = None
    biggest = min_jump
    for i in range(1, len(gt)):
        d = ((gt[i][1] - gt[i - 1][1]) ** 2 + (gt[i][2] - gt[i - 1][2]) ** 2) ** 0.5
        if d > biggest:
            biggest = d
            kidnap_t = gt[i][0]
    return kidnap_t


def main():
    if len(sys.argv) < 3:
        sys.stderr.write('usage: convergence_metrics.py gt.tum est.tum [threshold_m]\n')
        return 2
    gt = load(sys.argv[1])
    est = load(sys.argv[2])
    threshold = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3

    pairs = associate(gt, est)
    if not pairs:
        sys.stderr.write('[convergence] no associable samples\n')
        print('converged=0 reason=no_samples')
        return 1

    # If the robot was kidnapped mid-run, score the recovery from the teleport
    # onward -- the pre-kidnap tracking is not what we are measuring.
    kidnap_t = detect_kidnap(gt)
    if kidnap_t is not None:
        after = [p for p in pairs if p[0] >= kidnap_t]
        if after:
            pairs = after

    t0 = pairs[0][0]
    errs = [e for _, e in pairs]
    max_error = max(errs)
    final_error = errs[-1]

    # Earliest sample after which the error stays below the threshold for the
    # rest of the run. This is robust to a late transient only if it recovers;
    # a run that ends above threshold is reported as not converged.
    converge_idx = None
    worst_tail = 0.0
    for i in range(len(pairs) - 1, -1, -1):
        worst_tail = max(worst_tail, errs[i])
        if worst_tail < threshold:
            converge_idx = i
        else:
            break

    converged = converge_idx is not None
    if converged:
        ttc = pairs[converge_idx][0] - t0
        tail = errs[converge_idx:]
        post_rmse = (sum(e * e for e in tail) / len(tail)) ** 0.5
        ttc_s = '{:.2f}'.format(ttc)
        post_s = '{:.4f}'.format(post_rmse)
    else:
        ttc_s = 'nan'
        post_s = 'nan'

    kidnap_note = ' (from kidnap)' if kidnap_t is not None else ''
    human = (
        '[convergence] threshold={:.2f} m  samples={}{}\n'
        '  converged            : {}\n'
        '  time_to_converge_s   : {}\n'
        '  post_converge_rmse_m : {}\n'
        '  final_error_m        : {:.4f}\n'
        '  max_error_m          : {:.4f}\n'.format(
            threshold, len(pairs), kidnap_note, 'yes' if converged else 'NO',
            ttc_s if converged else 'n/a',
            post_s if converged else 'n/a',
            final_error, max_error))
    sys.stderr.write(human)

    kidnapped = 1 if kidnap_t is not None else 0
    print(
        'converged={} time_to_converge_s={} post_converge_rmse_m={} '
        'final_error_m={:.4f} max_error_m={:.4f} threshold_m={:.2f} kidnapped={}'.format(
            1 if converged else 0, ttc_s, post_s,
            final_error, max_error, threshold, kidnapped))
    return 0


if __name__ == '__main__':
    sys.exit(main())
