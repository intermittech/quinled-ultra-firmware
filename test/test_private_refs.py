# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Hardware is private: the repo's code, comments and docs name no component references (designators: a letter group
such as U, R or J and a three-digit number).
Scans every tracked text file outside the recorded evidence (test/evidence, experiments: build logs and reports of
their time) for reference-shaped tokens; identifiers that only look like one are listed with their reason.

    python test/test_private_refs.py [--src <firmware tree>]
"""
import argparse
import pathlib
import re
import subprocess
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
REF = re.compile(r'\b(?:U|R|C|Q|D|J|L|F|Y|TP|SW|FB|LED)[0-9]{3}[A-Z]?\b')
NOT_REFS = {'L065', 'L066'}                       # tools/flash_stores.py: record layout names, not designators
SKIP = ('test/evidence/', 'experiments/')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', type=pathlib.Path, default=FW)
    a = ap.parse_args()
    src = a.src.resolve()
    files = subprocess.check_output(['git', '-c', f'safe.directory={src.as_posix()}', '-C', str(src), 'ls-files'],
                                    text=True).splitlines() if (src / '.git').exists() else \
        [p.relative_to(src).as_posix() for p in src.rglob('*') if p.is_file()]
    hits = []
    for rel in files:
        if rel.startswith(SKIP) or '__pycache__' in rel:
            continue
        path = src / rel
        try:
            text = path.read_text(encoding='utf-8')
        except (UnicodeDecodeError, OSError):
            continue
        for n, line in enumerate(text.splitlines(), 1):
            line = line.split('# noqa')[0]                # (lint codes such as F401)
            for m in REF.finditer(line):
                if m.group(0) not in NOT_REFS:
                    hits.append(f'{rel}:{n}: {m.group(0)}')
    print(f'component references: {len(hits)}')
    for h in hits[:20]:
        print('  ' + h)
    sys.exit(1 if hits else 0)


if __name__ == '__main__':
    main()
