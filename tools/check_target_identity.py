# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Compare the twelve size-gate load images and all supplied bootloader images.
Usage: python tools/check_target_identity.py <baseline-dir> <candidate-dir> [--out evidence.json]
Build both trees with tools/size_gate.py --out <dir>; bootloaders may be <dir>/bl*/bl.bin.
ELF debug paths legitimately change; this checks binary bytes and Intel HEX bytes.
"""
import argparse
import hashlib
import json
from pathlib import Path
from size_gate import BUDGET

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('baseline',type=Path);p.add_argument('candidate',type=Path)
p.add_argument('--out',type=Path)
a=p.parse_args();rows=[]
paths=[Path(name)/('fw.'+ext) for name,*_ in BUDGET for ext in ('bin','hex')]
paths += [f.relative_to(a.baseline) for f in sorted(a.baseline.glob('bl*/bl.bin'))]
for rel in paths:
 old=(a.baseline/rel).read_bytes();new=(a.candidate/rel).read_bytes()
 rows.append({'image':rel.as_posix(),'bytes':len(new),'sha256':hashlib.sha256(new).hexdigest(),'baseline_sha256':hashlib.sha256(old).hexdigest(),'identical':new==old})
 print(rel, 'IDENTICAL' if new==old else 'DIFFERENT')
if a.out:a.out.write_text(json.dumps(rows,indent=2)+'\n')
raise SystemExit(any(not r['identical'] for r in rows))
