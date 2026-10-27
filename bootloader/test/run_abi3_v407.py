# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Run the CH32V407 ABI-3 bootloader matrix: test_bl_abi3_v407.py (transactions, interruptions, faults and the
first-boot option write, the status way), the boot path through each app's startup, the status-profile loop with
the protocol's C99 sender (test_bl_status_v407.py) and the main loop on the wire (test_bl_wire_v407.py),
for every bootloader ELF given (both compilers' builds when they differ); exits nonzero on any failure.

    python bootloader/test/run_abi3_v407.py --bl <xpack bl.elf> [--bl <wch bl.elf>] [--app <fw.elf> ...]
        [--proto <protocol checkout>] --evidence <dir>   (--proto: default as test_bl_status_v407.py)
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bl", type=Path, action="append", required=True)
    p.add_argument("--app", type=Path, action="append", default=[])
    p.add_argument("--proto", type=Path, default=Path(os.environ.get(
        "ULTRA_PROTOCOL_REPO", str(ROOT.parent / "quinled-ultra-protocol"))))
    p.add_argument("--evidence", type=Path, required=True)
    a = p.parse_args()
    bins = [b.with_suffix(".bin").read_bytes() for b in a.bl]
    identical = all(x == bins[0] for x in bins)
    a.evidence.mkdir(parents=True, exist_ok=True)
    here = Path(__file__).parent
    jobs = []
    for i, b in enumerate(a.bl[:1] if identical else a.bl):
        jobs.append((f"bl{i}-matrix", "test_bl_abi3_v407.py", [str(b)]))
        jobs += [(f"bl{i}-app{k}-startup", "test_bl_abi3_v407.py", [str(b), "--app", str(app)])
                 for k, app in enumerate(a.app)]
        jobs.append((f"bl{i}-status-loop", "test_bl_status_v407.py",
                     [str(b), str(a.evidence / f"status{i}"), "--proto", str(a.proto)]))
        jobs.append((f"bl{i}-wire", "test_bl_wire_v407.py", [str(b), str(a.evidence / f"wire{i}")]))
    rows = []
    for name, script, args in jobs:              # (sequential: the PC's CPU budget)
        with (a.evidence / f"{name}.log").open("w") as log:
            r = subprocess.run([sys.executable, str(here / script), *args],
                               cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=3600)
        rows.append(dict(test=name, exit=r.returncode))
        print(json.dumps(rows[-1]), flush=True)
    result = dict(identical_compilers=identical, jobs=rows)
    (a.evidence / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    raise SystemExit(any(r["exit"] for r in rows))


if __name__ == "__main__":
    main()
