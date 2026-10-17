# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""BOARD=3's real main loop, idle MMIO, and captured marker frames.

--release uses the actual LTO release ELF, descriptor and bootloader identity.
These execute target instructions, not physical peripheral timing.
"""
import argparse
from test_ud_emulated import FW, frame, wave, crc
from mainloop import Emu2
from flash_stores import FlashEmu

class Board(FlashEmu, Emu2):
    pass

p = argparse.ArgumentParser()
p.add_argument("--release", action="store_true")
p.add_argument("--case", choices=("version", "length", "hardware", "wrap"), default="version")
a = p.parse_args()
release = FW / "build/release/0.6.11-rc.14"
elf = release / "obj/app-ud-rev0/fw.elf" if a.release else FW / "build/r14-ud-api/fw.elf"

def boot(hw=3, ms=0, tick=0):
    e = Board(str(elf))
    e.poke(0x08010000, b"\xff" * 0xC000)
    e.mmio.update({0x40021000: 0x02000002, 0x40021004: 0x08, 0x40004400: 0xC0,
                  0x40010C08: 1 << 10, 0x40010808: 1 << 3})
    if a.release:
        desc = (release / "quinled-diff-power-ultra-digital-0.6.11-rc.14-ud-rev0.bin").read_bytes()[:64]
        e.poke(0x08002000, desc + b"\xff" * 192)
        e.poke(0x08000010, b"QDBL" + bytes([2 if hw else 0, hw, 0]) + bytes(9))
    e.wr(e.addr("ms_now"), 4, ms)
    e.wr(e.addr("tick_now"), 4, tick)
    base, size = e.sym("main")
    wfi = [q for q in range(base, base + size, 2) if e.rd(q, 4) == 0x10500073]
    assert len(wfi) == 1, wfi
    stop = set(wfi)
    pc, n = e.run_to(e.start("main"), stop, max_steps=2000000)
    print(f"BOARD=3 {'release' if a.release else 'standalone'} main, bootloader hw={hw}: WFI after {n} instructions", flush=True)
    return e, pc, stop

def led(e):
    return [w for w in e.writes if w[0] in (0x40010C10, 0x40010C14) and w[1] & ((1 << 15) | (1 << 31))]

def capture(e, data, tick):
    assert e.sym("dl")[1] == 2712
    w = wave(data)
    dl = e.addr("dl")
    e.poke(dl + 152, w)
    e.wr(dl + 84, 2, len(w) // 2)
    e.wr(e.addr("dl_ready"), 1, 1)
    e.wr(e.addr("dl_ready_tick"), 4, tick)

if a.case == "version":
    e, pc, stop = boot()
    print(f"boot LED writes: {led(e)}", flush=True)
    assert led(e), "control: the fixture observes the status LED"
    poll = bytearray(frame(value=4))
    poll[7] = 1  # IDENTITY part FEATURES (POLL args begin at slot byte 7).
    poll[30:32] = crc(poll[:30], 16, 0x1021, 0xFFFF).to_bytes(2, "big")
    capture(e, poll, 0)
    pc, _ = e.run_to(pc, stop, max_steps=100000)
    answer = e.peek(e.addr("ud_ans"), 32)
    assert answer[1] == 4 and crc(answer[:30], 16, 0x1021, 0xFFFF) == int.from_bytes(answer[30:], "big")
    features = int.from_bytes(answer[6:10], "big")
    print(f"IDENTITY reply: VERSION_BLINK advertised={bool(features & 0x400)}", flush=True)
    e.writes.clear()
    for ms in range(1, 5001):
        if ms in (1, 101, 201):
            capture(e, frame(kind=6, value=5, flags=0), ms * 20)
        e.wr(e.addr("ms_now"), 4, ms)
        e.wr(e.addr("tick_now"), 4, ms * 20)
        pc, _ = e.run_to(pc, stop, max_steps=100000)
    decoded = e.rd(e.addr("uds") + 10, 2)
    print(f"IDENTIFY 5: decoded frames={decoded}; 5000 ms of main-loop service; LED writes={led(e)}", flush=True)
    assert decoded == 4, "the IDENTITY poll and all three IDENTIFY slots reached ud_frame"
    if features & 0x400 and not led(e):
        raise SystemExit("FAIL: advertised VERSION_BLINK produced no LED transition")
elif a.case == "length":
    failures = 0
    for extra in (0, 1, 2, 8):
        e, pc, stop = boot()
        f = frame(kind=6, value=7, flags=0) + bytes(extra)
        capture(e, f, 0)
        pc, _ = e.run_to(pc, stop, max_steps=100000)
        planned = e.peek(e.addr("vdip") + 15, 1)[0]
        decoded = e.rd(e.addr("uds") + 10, 2)
        print(f"UD release main: {len(f) * 8}-bit capture, IDENTIFY 7 prefix, decoded={decoded}, planned={planned}", flush=True)
        if bool(planned) != (extra == 0):
            failures += 1
            print("FAIL: only an exact valid frame may set the planned-cut flag", flush=True)
    print(f"R14 UD release length: 4 checks, {failures} failures", flush=True)
    raise SystemExit(bool(failures))
elif a.case == "hardware":
    assert a.release, "--release required for the bootloader identity check"
    for hw in (0, 1, 2, 3, 255):
        e, pc, stop = boot(hw)
        on = e.rd(e.addr("dl_on"), 1)
        dma = [(reg, val) for reg, val in e.writes if reg in (0x4002001C, 0x40020030) and val & 1]
        print(f"bootloader hw={hw}: delay line on={on}, DMA channel enable writes={dma}", flush=True)
        assert bool(on) == (hw in (0, 3))
        assert bool(dma) == (hw in (0, 3))
    print("R14 UD hardware: 5 identities, 10 checks, 0 failures", flush=True)
else:
    snapshots = []
    for start_ms, start_tick in ((0, 0), (0xFFFFF000, 0xFFFFF000)):
        e, pc, stop = boot(ms=start_ms, tick=start_tick)
        for delta in range(1, 11002):
            ms, tick = (start_ms + delta) & 0xFFFFFFFF, (start_tick + delta * 20) & 0xFFFFFFFF
            if delta == 1:
                capture(e, frame(kind=6, value=7, flags=0), tick)
            e.wr(e.addr("ms_now"), 4, ms)
            e.wr(e.addr("tick_now"), 4, tick)
            pc, _ = e.run_to(pc, stop, max_steps=100000)
            if delta in (10000, 10001):
                planned = e.rd(e.addr("vdip") + 15, 1)
                print(f"start_ms=0x{start_ms:08x}, elapsed={delta}: planned={planned}", flush=True)
                assert planned == (delta < 10001)
        snap = tuple(e.rd(e.addr(n), 4) for n in ("up_s", "ovr_late"))
        snap += (e.rd(e.addr("en") + 4, 4), e.rd(e.addr("vdip") + 15, 1))
        snapshots.append(snap)
        print(f"start_ms=0x{start_ms:08x}, 11001 ms: uptime/late/powered/planned={snap}", flush=True)
    assert snapshots[0] == snapshots[1] == (11, 0, 11, 0)
    print("R14 UD main wrap: ms and 20-kHz tick wrap, notice expiry, lifetime seconds: passed", flush=True)
