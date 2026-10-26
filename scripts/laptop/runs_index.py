#!/usr/bin/env python3
"""Add a run to the repo's runs index (code-side record; NEVER bags or video in git).
    runs_index.py <run_dir> [--where "G:/My Drive/hardleel_runs"]
-> runs/INDEX.md row (folder, date, commit, notes summary, where the data lives) and runs/<run>/ with the small
files only: NOTES.md, git_commit.txt, mission CSV/JSON logs, verify.json, saved map .yaml/.pgm (< 2 MB each)."""
import os
import shutil
import sys

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
SMALL = ('.md', '.txt', '.csv', '.json', '.yaml', '.pgm', '.png')


def main():
    run_dir = os.path.expanduser(sys.argv[1]).rstrip('/')
    where = sys.argv[sys.argv.index('--where') + 1] if '--where' in sys.argv else 'laptop ~/runs (not backed up yet)'
    run = os.path.basename(run_dir)
    out = os.path.join(REPO, 'runs', run)
    os.makedirs(out, exist_ok=True)
    copied = 0
    for root, _dirs, files in os.walk(run_dir):
        if os.sep + 'bags' in root or os.sep + 'video' in root or os.sep + 'stills' in root:
            continue
        for f in files:
            p = os.path.join(root, f)
            if f.endswith(SMALL) and os.path.getsize(p) < 2e6:
                dst = os.path.join(out, os.path.relpath(p, run_dir))
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.copy(p, dst)
                copied += 1
    commit = open(os.path.join(run_dir, 'git_commit.txt')).readline().strip()[:10] if os.path.exists(
        os.path.join(run_dir, 'git_commit.txt')) else '?'
    notes = ''
    if os.path.exists(os.path.join(run_dir, 'NOTES.md')):
        lines = [l.strip('- \n') for l in open(os.path.join(run_dir, 'NOTES.md')) if l.startswith('- what happened')]
        notes = lines[0].split(':', 1)[-1].strip() if lines else ''
    idx = os.path.join(REPO, 'runs', 'INDEX.md')
    if not os.path.exists(idx):
        open(idx, 'w').write('# Runs index\n\n| run | date | commit | notes | data |\n|---|---|---|---|---|\n')
    date = run[4:12] if run.startswith('run_') else '?'
    rows = open(idx).read()
    if f'| {run} |' not in rows:
        open(idx, 'a').write(f'| {run} | {date[:4]}-{date[4:6]}-{date[6:8]} | {commit} | {notes or "-"} | {where} |\n')
    print(f'{run}: {copied} small files -> {out}; INDEX.md updated')


if __name__ == '__main__':
    main()
