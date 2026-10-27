# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Chain-line sweep: <pad> [effect] [npx] [reset_us] [chip] [fade].
ELF overrides build/pad-N/fw.elf. Fade: 0 alone, positive effect = fade out,
negative effect = fade in; same effect = palette fade. Each run tests 8 phases
and releases at +150/+5 us. Returns nonzero if any frame or poll is lost.
"""
import concurrent.futures
import os
import sys
sys.path.insert(0, 'test')
import test_ur_digital_m_chain_budget as B


def main():
    pad = int(sys.argv[1])
    effect = int(sys.argv[2]) if len(sys.argv) > 2 else 18
    npx = int(sys.argv[3]) if len(sys.argv) > 3 else 300
    reset_us = int(sys.argv[4]) if len(sys.argv) > 4 else 300
    chip = int(sys.argv[5]) if len(sys.argv) > 5 else 0
    fade = int(sys.argv[6]) if len(sys.argv) > 6 else 0
    elf = os.environ.get('ELF') or 'build/pad-%d/fw.elf' % pad
    jobs = []
    for rel in (150, 5):
        for ph in range(0, 50, 7):
            jobs.append(("rel +%d phase %d" % (rel, ph), elf, reset_us, ph, rel, npx, effect, chip, fade))
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
    print("effect %d chip %d px %d fade %d reset %d pad %d: %d of %d runs fail; slowest classification %.0f us; answers %s" % (
        effect, chip, npx, fade, reset_us, pad, fails, len(jobs), worst_c,
        "all in window" if all(v is not None and 1000 <= v <= 1080 for v in ans) else "some out"))
    return bool(fails)


if __name__ == "__main__":
    raise SystemExit(main())
