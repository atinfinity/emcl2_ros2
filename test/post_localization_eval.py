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
Post the localization-evaluation result to the pull request.

Uploads the evo APE plots to the `ci-assets` branch (so they render inline)
and posts/updates a single sticky comment on the PR with one section per
trajectory (APE stats, peak speeds, and plots). Runs from the localization CI
job on same-repo pull requests. Uses only the GitHub REST API via urllib, so it
needs no extra tooling in the container.

Usage: post_localization_eval.py "Label=/path/to/output_dir" [...]
  Each result dir holds ape_stats.txt / motion_stats.txt / trajectory_map.png /
  ape_plot_raw.png. With no args it falls back to a single unlabelled section
  from the OUTPUT_DIR env var.

Env: GITHUB_TOKEN, GITHUB_REPOSITORY (owner/repo), PR_NUMBER, GITHUB_RUN_ID.
"""

import base64
import json
import os
import re
import sys
import urllib.error
import urllib.request

API = 'https://api.github.com'
ASSETS_BRANCH = 'ci-assets'
MARKER = '<!-- evo-localization-eval -->'


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


def upload_asset(repo, token, local_path, remote_path):
    """Create or update a file on the ci-assets branch; return its raw URL."""
    with open(local_path, 'rb') as f:
        content = base64.b64encode(f.read()).decode()
    status, cur = api(
        'GET', f'/repos/{repo}/contents/{remote_path}?ref={ASSETS_BRANCH}', token)
    payload = {
        'message': f'update {remote_path}',
        'content': content,
        'branch': ASSETS_BRANCH,
    }
    if status == 200 and 'sha' in cur:
        payload['sha'] = cur['sha']
    st, resp = api('PUT', f'/repos/{repo}/contents/{remote_path}', token, payload)
    if st not in (200, 201):
        print(f'asset upload failed ({st}): {resp}', file=sys.stderr)
        sys.exit(1)
    return f'https://raw.githubusercontent.com/{repo}/{ASSETS_BRANCH}/{remote_path}'


def read_ape(out):
    raw = open(os.path.join(out, 'ape_stats.txt')).read().splitlines()
    start = next((i for i, ln in enumerate(raw) if ln.startswith('APE')), 0)
    return '\n'.join(
        ln for ln in raw[start:]
        if 'QStandardPaths' not in ln and 'Plot saved' not in ln).strip()


def section(repo, token, pr, label, out):
    """Build the markdown block for one trajectory's result (uploads its plots)."""
    slug = re.sub(r'[^a-z0-9]+', '', label.lower()) or 'run'
    parts = []
    if label:
        parts.append(f'### {label}\n\n')
    try:
        parts.append(f'```\n{read_ape(out)}\n```\n\n')
    except OSError:
        parts.append('_APE stats unavailable (run did not produce them)._\n\n')

    motion_path = os.path.join(out, 'motion_stats.txt')
    if os.path.exists(motion_path):
        text = open(motion_path).read().strip()
        if text:
            parts.append(f'**Peak speeds / path length (ground truth):**\n\n```\n{text}\n```\n\n')

    traj_path = os.path.join(out, 'trajectory_map.png')
    if os.path.exists(traj_path):
        url = upload_asset(repo, token, traj_path, f'pr{pr}_{slug}_traj_map.png')
        parts.append(
            '**Trajectory over the occupancy map (dashed = ground truth, '
            f'estimate colored by APE):**\n\n![{slug} trajectory on map]({url})\n\n')

    raw_path = os.path.join(out, 'ape_plot_raw.png')
    if os.path.exists(raw_path):
        url = upload_asset(repo, token, raw_path, f'pr{pr}_{slug}_ape_raw.png')
        parts.append(f'**APE over time:**\n\n![{slug} APE over time]({url})\n\n')
    return ''.join(parts)


def main():
    token = os.environ['GITHUB_TOKEN']
    repo = os.environ['GITHUB_REPOSITORY']
    pr = os.environ['PR_NUMBER']
    run_id = os.environ.get('GITHUB_RUN_ID', '')

    # "Label=dir" pairs; fall back to a single unlabelled OUTPUT_DIR section.
    runs = []
    for arg in sys.argv[1:]:
        label, _, path = arg.partition('=')
        runs.append((label.strip(), path.strip()))
    if not runs:
        runs = [('', os.environ['OUTPUT_DIR'])]

    blocks = ''.join(section(repo, token, pr, label, out) for label, out in runs)
    body = (
        f'{MARKER}\n'
        '## localization CI: APE (emcl2 vs Gazebo ground truth)\n\n'
        'Accuracy across trajectory shapes -- straight lawnmower, sustained '
        'arc, and in-place rotation.\n\n'
        f'{blocks}'
        f'<sub>generated by CI run {run_id}</sub>'
    )

    # Sticky comment: update the existing marker comment if present, else create.
    _, comments = api('GET', f'/repos/{repo}/issues/{pr}/comments?per_page=100', token)
    existing = next(
        (c for c in comments if isinstance(c, dict) and MARKER in c.get('body', '')), None)
    if existing:
        api('PATCH', f'/repos/{repo}/issues/comments/{existing["id"]}', token, {'body': body})
        print(f'updated comment {existing["id"]}')
    else:
        api('POST', f'/repos/{repo}/issues/{pr}/comments', token, {'body': body})
        print('created comment')


if __name__ == '__main__':
    main()
