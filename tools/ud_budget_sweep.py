# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""the chain-line budget with a heavy effect (Pacifica, 300 px) and a render step padded by N loop turns (3 cycles each): usage sweep_budget.py <pad> [effect] [npx]"""
import concurrent.futures
import os
import sys
sys.path.insert(0, 'test')
import test_ud_chain_budget as B


def main():
    pad = int(sys.argv[1])
    effect = int(sys.argv[2]) if len(sys.argv) > 2 else 18
    npx = int(sys.argv[3]) if len(sys.argv) > 3 else 300
    elf = os.environ.get('ELF') or 'build/pad-%d/fw.elf' % pad
    jobs = []
    for rel in (150, 5):
        for ph in range(0, 50, 7):
            jobs.append(("rel +%d phase %d" % (rel, ph), elf, 300, ph, rel, npx, effect))
    fails = 0
    worst_c = 0.0
    ans = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=min(len(jobs), os.cpu_count() or 1, 8)) as pool:
        for label, r in pool.map(B.job, jobs):
            ok = (r["frames"] == r["sent"] and r["burst_ok"] and r["nonultra"] == 0 and
                  all(v is not None and 1000.0 <= v <= 1080.0 for v in r["answers"].values()))
            fails += not ok
            worst_c = max(worst_c, r["worst"])
            for k, v in r["answers"].items():
                ans.append(v)
            if not ok:
                print("   FAIL", label, r["frames"], "of", r["sent"], "nonultra", r["nonultra"], r["answers"], "slowest %.0f" % r["worst"])
    chunk = 32 * int(os.environ.get('CPP', '801')) + 3 * pad
    print("pad %6d  render step ~%7d cycles = %5.0f us: %d of %d runs fail; slowest classification %.0f us; answers %s" % (
        pad, chunk, chunk / 144.0, fails, len(jobs), worst_c, "all in window" if all(v is not None and 1000 <= v <= 1080 for v in ans) else "some out"))


if __name__ == "__main__":
    main()
