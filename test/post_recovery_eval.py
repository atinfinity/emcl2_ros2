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
Post the recovery / global-localization convergence metrics to the pull request.

Informational only: reports whether and how fast emcl2 recovers a correct pose
from a wrong / uniform belief. It never gates the PR -- a non-convergence is a
result, not a failure. Posts/updates a single sticky comment via the GitHub REST
API (urllib only, no extra tooling).

Args: one or more "Label=/path/to/output_dir" entries, each holding a
convergence.txt produced by convergence_metrics.py.

Env: GITHUB_TOKEN, GITHUB_REPOSITORY (owner/repo), PR_NUMBER, GITHUB_RUN_ID.
"""

import glob
import json
import os
import statistics
import sys
import urllib.error
import urllib.request

API = 'https://api.github.com'
MARKER = '<!-- recovery-localization-eval -->'


def api(method, path, token, payload=None):
    url = path if path.startswith('http') else API + path
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header('Authorization', f'Bearer {token}')
    req.add_header('Accept', 'application/vnd.github+json')
    req.add_header('X-GitHub-Api-Version', '2022-11-28')
    try:
        with urllib.request.urlopen(req) as resp:
            body = resp.read()
            return resp.status, (json.loads(body) if body else {})
    except urllib.error.HTTPError as e:
        return e.code, (json.loads(e.read() or b'{}'))


def parse(path):
    """Parse a convergence.txt 'key=value key=value ...' line into a dict."""
    try:
        line = open(path).read().split('\n')[0].strip()
    except OSError:
        return {}
    out = {}
    for tok in line.split():
        if '=' in tok:
            k, v = tok.split('=', 1)
            out[k] = v
    return out


def collect(parent):
    """Parse every run*/convergence.txt under parent (or a single one directly)."""
    runs = []
    for f in sorted(glob.glob(os.path.join(parent, 'run*', 'convergence.txt'))):
        m = parse(f)
        if m:
            runs.append(m)
    if not runs:
        m = parse(os.path.join(parent, 'convergence.txt'))
        if m:
            runs.append(m)
    return runs


def _floats(runs, key, only_converged=False):
    out = []
    for r in runs:
        if only_converged and r.get('converged') != '1':
            continue
        try:
            out.append(float(r.get(key, 'nan')))
        except ValueError:
            pass
    return out


def aggregate(runs):
    """Success rate plus median time-to-converge / final error over N runs."""
    converged = sum(1 for r in runs if r.get('converged') == '1')
    ttc = _floats(runs, 'time_to_converge_s', only_converged=True)
    finals = _floats(runs, 'final_error_m')
    return {
        'n': len(runs),
        'converged': converged,
        'med_ttc': statistics.median(ttc) if ttc else None,
        'med_final': statistics.median(finals) if finals else None,
    }


def row(label, a):
    if a['n'] == 0:
        return f'| {label} | 0 | ⚠️ no result | – | – |'
    rate = 100.0 * a['converged'] / a['n']
    ttc = f'{a["med_ttc"]:.1f} s' if a['med_ttc'] is not None else '–'
    fin = f'{a["med_final"]:.3f} m' if a['med_final'] is not None else '–'
    return f'| {label} | {a["n"]} | {a["converged"]}/{a["n"]} ({rate:.0f}%) | {ttc} | {fin} |'


def main():
    token = os.environ['GITHUB_TOKEN']
    repo = os.environ['GITHUB_REPOSITORY']
    pr = os.environ['PR_NUMBER']
    run_id = os.environ.get('GITHUB_RUN_ID', '')

    entries = []
    threshold = '?'
    for arg in sys.argv[1:]:
        label, _, out_dir = arg.partition('=')
        runs = collect(out_dir)
        if runs:
            threshold = runs[0].get('threshold_m', threshold)
        entries.append((label, aggregate(runs)))
    if not entries:
        print('no scenarios given', file=sys.stderr)
        return 2

    rows = '\n'.join(row(lbl, a) for lbl, a in entries)
    body = (
        f'{MARKER}\n'
        '## localization CI: recovery / global localization\n\n'
        f'Convergence of emcl2 to the true pose from a wrong / uniform belief '
        f'(threshold = {threshold} m), aggregated over N runs. Medians are over '
        f'converged runs (time) and all runs (final error). Informational — does '
        f'not gate the PR.\n\n'
        '| scenario | runs | converged | median time to converge '
        '| median final error |\n'
        '|---|---|---|---|---|\n'
        f'{rows}\n\n'
        f'<sub>generated by CI run {run_id}</sub>'
    )

    _, comments = api('GET', f'/repos/{repo}/issues/{pr}/comments?per_page=100', token)
    existing = next(
        (c for c in comments if isinstance(c, dict) and MARKER in c.get('body', '')), None)
    if existing:
        api('PATCH', f'/repos/{repo}/issues/comments/{existing["id"]}', token, {'body': body})
        print(f'updated comment {existing["id"]}')
    else:
        api('POST', f'/repos/{repo}/issues/{pr}/comments', token, {'body': body})
        print('created comment')
    return 0


if __name__ == '__main__':
    sys.exit(main())
