# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The Diff-Power-UR-Digital-M (V203) at any position of a CHAIN line, on the COMPILED BOARD=3 image: the relay window and the gate's slot plan
with the main loop PREEMPTED by the 20 kHz tick (test_ur_digital_m_chain_budget.py's run_once: 7,200 cycles a tick, the pins followed).

Frames carry this board's slot first and nslots - 1 NOP slots behind it (the boards behind); the POLLs have R slots and this board's index (255 =
RELAY_ONLY). Checked per run: the worst tick + 60 <= 7,200 cycles; every frame classified, none a no-Ultra frame; answers inside slot 0's window
(index 0) or none (RELAY_ONLY); the relay window: DE high from the turnaround (t0 + 450 .. 610 us) to forward-again (W + 5 .. 80 us, W = 1000 + R x 500);
the gate: one re-arm in every gap after the first, >= 200 us after the line's last edge (after the window behind an RW frame), none inside a frame.

The matrix (16 runs): 8 slots, R 8, index 0 (light, Lava Lamp, Fire; phases 0 / 25); 8 slots, R 8, RELAY_ONLY (light, Lava Lamp); the chain end, 1 slot,
R 1 (light, Lava Lamp); 8 slots, R 8, index 0, the release at +60 us (light).

    python test/test_ur_digital_m_relay_budget.py <fw.elf> [workers=4]
"""
import concurrent.futures
import os
import sys
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / "test"))
sys.path.insert(0, str(FW / "tools"))
import test_ur_digital_m_chain_budget as T  # noqa: E402
from ur_digital_m_target import offsets  # noqa: E402

NSLOTS = 1
_frame = T.Line.frame


def frame_n(self, slot_bytes, rw=0, kind="frame", gap_us=500):
    """this board's slot, then nslots - 1 NOP slots (the boards behind it)"""
    if NSLOTS > 1 and len(slot_bytes) == 32:
        slot_bytes = slot_bytes + b"".join(T.slot(bytes([0x00, 0, 0x40 + i])) for i in range(NSLOTS - 1))
    return _frame(self, slot_bytes, rw, kind, gap_us)


T.Line.frame = frame_n
CASES = {
    "light": dict(rel_us=150, npx=40, effect=4),
    "light60": dict(rel_us=60, npx=40, effect=4),
    "lava": dict(rel_us=150, npx=600, effect=31),
    "fire0": dict(rel_us=150, npx=600, effect=16, chip=0),
}
MATRIX = [(8, 8, 0, ("light", "lava", "fire0")), (8, 8, 255, ("light", "lava")), (1, 1, 0, ("light", "lava")), (8, 8, 0, ("light60",))]
PHASES = (0, 25)


def job(a):
    global NSLOTS
    elf, n, r, idx, name, phase = a
    NSLOTS = n
    T.R_SLOTS = r
    T.POLL_INDEX = idx
    return a, T.run_once(elf, offsets(elf), 500, phase, **CASES[name])


def check_relay(rl, r):
    """the relay windows' DE and the gate's re-arms -> (turns, errors, (the least time from a re-arm to the next frame, re-arms a gap))"""
    ends, starts, errs, turns = rl["ends_us"], rl["starts"], [], []
    W = 1000 + r * 500
    for kind, t in ends:
        if not kind.startswith("poll"):
            continue
        on = [x for x in rl["de_on"] if t < x < t + 2000]
        off = [x for x in rl["de_off"] if t + 900 < x < t + W + 2000]
        if not on or not 450 <= on[0] - t <= 610:
            errs.append(f"{kind}: DE on {[round(x - t) for x in on]} us after t0")
        if not off or not W + 5 <= off[-1] - t <= W + 80:
            errs.append(f"{kind}: DE off {[round(x - t) for x in off]} us after t0 (W {W})")
        turns.append((kind, round(on[0] - t) if on else None, round(off[-1] - t) if off else None))
    rearm = [t for t, m in rl["gate_ev"] if m == 3]
    if not rearm:
        return turns, ["no re-arm"], (0, 0)
    first = next(i for i, s in enumerate(starts) if s > rearm[0])
    most, worst_gap = 0, 10 ** 9
    for i in range(first, len(starts)):
        prev_end = ends[i - 1][1]
        lo = prev_end + (W + 40 if ends[i - 1][0].startswith("poll") else 0)   # the line's last edge before the gap's quiet LOW
        inside = [t for t in rearm if starts[i] <= t <= ends[i][1]]
        gap = [t for t in rearm if prev_end < t < starts[i]]
        if inside or not gap or gap[-1] - lo < 200 or any(t < lo for t in gap):
            errs.append(f"frame {i} ({ends[i][0]}): re-arms in its gap {[round(t - prev_end) for t in gap]} us after the last frame, inside {len(inside)}")
        else:
            worst_gap = min(worst_gap, starts[i] - gap[-1])
            most = max(most, len(gap))
    return turns, errs, (worst_gap, most)


def main():
    elf = sys.argv[1]
    workers = max(1, min(4, int(sys.argv[2]) if len(sys.argv) > 2 else 4))
    jobs = [(elf, n, r, idx, nm, ph) for n, r, idx, names in MATRIX for nm in names for ph in PHASES]
    print(f"{len(jobs)} runs (slots per frame, POLL R, index: 255 = RELAY_ONLY); image {elf}")
    worst_isr = fails = 0
    with concurrent.futures.ProcessPoolExecutor(max_workers=workers) as pool:
        for (_, n, r, idx, nm, ph), res in pool.map(job, jobs):
            turns, errs, margin = check_relay(res["relay"], r)
            ans = res["answers"]
            if idx == 255:
                ans_ok = all(v is None for v in ans.values())
            else:
                ans_ok = all(v is not None and T.RW_FIRST - T.RW_EARLY0 <= v <= T.RW_FIRST + T.RW_LATE for v in ans.values())
            ok = (res["max_isr"] + 60 <= 7200 and res["frames"] == res["sent"] and res["burst_ok"] and res["nonultra"] == 0 and ans_ok and not errs)
            a = ", ".join(f"{k} {'none' if v is None else '%+.0f us' % (v - 1000.0)}" for k, v in ans.items())
            print(f"  {n} slots, R {r}, index {idx}, {nm}, phase {ph}: ISR {res['max_isr']}+60/7200; {res['frames']}/{res['sent']} classified, "
                  f"{res['nonultra']} no-Ultra; answers {a}; relay (turn, forward) us {turns}; the next frame >= {margin[0]:.0f} us after the last "
                  f"re-arm (at most {margin[1]} a gap)  {'ok' if ok else 'FAIL'}" + "".join("\n      " + x for x in errs[:6]))
            worst_isr = max(worst_isr, res["max_isr"])
            fails += not ok
    print(f"relay budget: worst ISR {worst_isr} (+60 = {worst_isr + 60} of 7200); {'ok' if not fails else 'FAIL, %d runs' % fails}")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
