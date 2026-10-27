# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 bootloader's status profile (docs/update.md) in a closed loop with the protocol repository's C99 sender.

The real bootloader ELF runs in rv32emu on abi3_model_v407.py's flash, with its answer UART modeled (bit times from
BRR at the true clock, TXE / TC, DE): the sender (built from the pinned protocol commit with MSVC, unmodified) emits
SELECT, ANNOUNCE, training PINGs, UPD_STATUS polls, DATA within credit, END and the acknowledgement; the bootloader
receives each frame at its end on its own clock (frame_t0 in local ticks: the true HCLK, off nominal by the HSI's
error), answers every poll it is locked for, and its answers go back into ultra_update_answer, which decodes them
with the sender's checks (state / credit / mask / accepted / committed consistency). Checked:
- the transfer completes: the committed image, its VERIFIED record, the ACK PING's reboot request;
- every answer's timing on the host clock (docs/timing.md, marker port): enable 270..900 us after the poll's end,
  first start bit 2..10 us later, inter-byte gaps <= 8 us, DE low within 5 us of the last stop bit, the burst over
  by 900 + pitch - 30 us, the baud within 1.5 % of nominal;
- no answer before 1 s of training (rate lock), none for another UID, and an unselected bootloader (another UID on the
  same line) never answers and never mutates flash;
- each flash operation starts only after its BUSY answer's window, and a lost answer (dropped once) is retried by the
  sender without a repeated erase.
Rates 1 Mbaud / 500 / 250 kbaud, HSI at -4.5 % / +0 % / +2.2 % of the 20 MHz nominal, two body lengths.

    python bootloader/test/test_bl_status_v407.py <bl.elf> <out dir> [--proto <protocol checkout>]
"""
import argparse
import ctypes
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import APP, BODY, MCU, image                 # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
PIN = "d5f17d2"
NOMINAL = 200.0                                                   # MHz: ticks per us at a nominal HSI
OFFSETS = r'''
#include "bl_core.h"
#include <stddef.h>
void b_init(void) { }
unsigned b_off(unsigned i)
{
    static const unsigned o[] = { offsetof(bl_sess_t, st.frame_t0), offsetof(bl_sess_t, st.reply),
        offsetof(bl_sess_t, st.answer), offsetof(bl_sess_t, st.reply_at), offsetof(bl_sess_t, st.brr),
        offsetof(bl_sess_t, st.locked), offsetof(bl_sess_t, st.op), offsetof(bl_sess_t, st.op_due),
        offsetof(bl_sess_t, st.op_at), offsetof(bl_sess_t, st.committed), offsetof(bl_sess_t, mode),
        offsetof(bl_sess_t, exit_result), offsetof(bl_sess_t, st.selected), offsetof(bl_sess_t, state) };
    return o[i];
}
'''
FRAME_T0, REPLY, ANSWER, REPLY_AT, BRR, LOCKED, OP, OP_DUE, OP_AT, COMMITTED, MODE, EXIT_RESULT, SELECTED, STATE = \
    range(14)
USART7 = 0x40001C00
GPIOA_BSHR, GPIOA_BCR, GPIOB_INDR = 0x40010810, 0x40010814, 0x40010C08


def msvc():
    vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / \
        "Microsoft Visual Studio/Installer/vswhere.exe"
    vs = Path(subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-requires",
                                       "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property",
                                       "installationPath"], text=True).strip())
    return vs / "VC/Auxiliary/Build/vcvars64.bat"


def build_sender(proto, out):
    src = out / "protocol"
    src.mkdir(parents=True, exist_ok=True)
    names = subprocess.check_output(["git", "-c", f"safe.directory={Path(proto).as_posix()}", "-C", str(proto),
                                     "ls-tree", "-r", "--name-only", PIN, "sender/src"], text=True).splitlines()
    for name in names:
        if name.endswith((".c", ".h")):
            (src / Path(name).name).write_bytes(subprocess.check_output(
                ["git", "-c", f"safe.directory={Path(proto).as_posix()}", "-C", str(proto), "show", f"{PIN}:{name}"]))
    dll = out / "status_sender.dll"
    cmd = out / "build_sender.cmd"
    cmd.write_text(f'@echo off\ncall "{msvc()}" >nul\ncl /nologo /O2 /W3 /LD /D_CRT_SECURE_NO_WARNINGS /I"{src}" '
                   f'"{Path(__file__).parent / "status_sender.c"}" ' +
                   " ".join(f'"{p}"' for p in sorted(src.glob("*.c"))) + f' /Fe:"{dll}" /Fo"{out}\\\\"\n')
    with (out / "build_sender.log").open("w") as log:
        subprocess.run(["cmd", "/c", str(cmd)], cwd=out, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=300)
    return ctypes.CDLL(str(dll))


def build_offsets(out):
    gcc = Path(os.environ["LOCALAPPDATA"]) / "xPacks/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc.exe"
    (out / "st_offsets.c").write_text(OFFSETS)
    elf = out / "st_offsets.elf"
    subprocess.run([str(gcc), "-march=rv32imc_zicsr_zifencei", "-mabi=ilp32", "-Os", "-nostdlib", "-DCH32V407",
                    "-DBL_HW_ID=3", "-DMCU_CH32V40X=1", "-I", str(ROOT / "src/mcu/ch32v40x"),
                    "-I", str(ROOT / "src/core"), "-I", str(ROOT / "bootloader"),
                    "-Wl,-e,b_init", "-T", str(ROOT / "test/fx_rv.ld"), str(out / "st_offsets.c"), "-o", str(elf)],
                   check=True, timeout=120)
    import rv32emu
    e = rv32emu.Emu(str(elf))
    offs = []
    for i in range(14):
        e.call("b_off", [i])
        offs.append(e.ret)
    return offs


class Board(MCU):
    """the bootloader with its answer UART, DE and DIN; local clock = `mhz` ticks per local us"""
    def __init__(self, elf, uid, mhz, **kw):
        super().__init__(elf, **kw)
        self.uid = uid
        self.mhz = mhz
        self.tx_end, self.buffered, self.bytes, self.de = 0, None, [], []
        self.mmio[0xE000F000] = 0x25                                   # SysTick running (hw_full_init did it)

    def bit_ticks(self):
        return 2 * (self.mmio.get(USART7 + 8, 1) or 1)                  # PCLK1 = HCLK / 2

    def shift(self):
        t = int(self.timer)
        if self.buffered is not None and t >= self.tx_end:
            self.bytes.append((self.tx_end, self.buffered))
            self.tx_end += 10 * self.bit_ticks()
            self.buffered = None

    def rd(self, a, n):
        if 0x1FFFF7E8 <= a < 0x1FFFF7F4:
            o = a - 0x1FFFF7E8
            return int.from_bytes(self.uid[o:o + n], "little")
        if a == USART7:
            self.shift()
            t = int(self.timer)
            return (0x80 if self.buffered is None else 0) | (0x40 if self.buffered is None and t >= self.tx_end else 0)
        if a == GPIOB_INDR:
            return 1 << 4                                                # the line released (HIGH)
        return super().rd(a, n)

    def wr(self, a, n, v):
        t = int(self.timer)
        if a == USART7 + 4:
            self.shift()
            if self.buffered is None and t >= self.tx_end:
                self.bytes.append((t, v & 255))
                self.tx_end = t + 10 * self.bit_ticks()
            else:
                assert self.buffered is None, "UART overrun"
                self.buffered = v & 255
            return
        if a == GPIOA_BSHR and v & (1 << 11):
            self.de.append(("on", t))
        if a == GPIOA_BCR and v & (1 << 11):
            self.de.append(("off", t))
        super().wr(a, n, v)

    def to_tick(self, tick):
        if tick / self.mhz > self.now_us:
            self.advance(tick / self.mhz)

    def ms(self):
        return int(self.timer / 195200)


def run(lib, elf, offs, out, rate, hsi, length, drop=None, observer=False):
    true_mhz = NOMINAL * (1 + hsi)                                       # HCLK of this board (ticks per host us)
    uid = bytes(range(0x31, 0x3d))
    other = bytes(range(0x51, 0x5d))
    b = Board(elf, uid, true_mhz)
    old = image(b.target, 772, 0x00030000)
    b.installed(old)
    bl_info = bytes(b.flash[0x10:0x30])
    obs = Board(elf, other, true_mhz) if observer else None
    if obs:
        obs.installed(old)
        obs_before = bytes(obs.flash)
    new = image(b.target, length, 0x00030100)
    for m in (b, obs) if obs else (b,):
        m.call("bl_app_read", [0x20006000])
        m.call("bl_sess_init", [m.addr("sess"), 3, 0x20006000, 0])        # BL_ACT_SESSION: the app asked
    host0, host = 2_000_000, 2_000_000
    rc = lib.s_begin(new, len(new), bl_info, uid, 9, rate, 0, ctypes.c_longlong(host))
    assert rc == 0, rc
    assert lib.s_plan(0) == 1, "the sender must plan the status profile for this target"
    sess = b.addr("sess")
    frame = (ctypes.c_ubyte * 48)()
    events = dict(frames=0, polls=0, answers=0, silent_polls=0, busy=0, errors=[], first_answer_host=None)
    pitch = (335 << rate) + 165
    entry_seen = False
    busy_window_end = 0
    first_frame_host = None
    for _ in range(200000):
        st = lib.s_state()
        if lib.s_boot_complete() or lib.s_failed():
            break
        host = max(host, lib.s_due())
        n = lib.s_next(ctypes.c_longlong(host), frame)
        if n == 0:
            host = max(host + 1, lib.s_due())
            continue
        assert n == 48, (n, lib.s_error())
        f = bytes(frame)
        if f[0] == 0x87 and st in (0, 1):
            continue                                                     # the app's SELECT (its own test)
        if f[0] == 0x81 and st in (1, 2):
            continue                                                     # the app's ANNOUNCE: it reboots
        entry_seen = True
        t_end = struct.unpack(">I", f[43:47])[0]                         # host us of the frame's end
        if first_frame_host is None:
            first_frame_host = t_end
        local = int((t_end - host0) * true_mhz) + 10_000_000
        for m in (b, obs) if obs else (b,):
            mo = m
            sess_m = m.addr("sess")
            # flash work due before this frame first, at its own time (the main loop polls continuously)
            for _ in range(4):
                due = mo.peek(sess_m + offs[OP_DUE], 1)[0]
                at = struct.unpack("<I", mo.peek(sess_m + offs[OP_AT], 4))[0]
                if not due or at >= local:
                    break
                n_ops = len(mo.ops)
                mo.to_tick(at)
                mo.call("bl_sess_poll", [sess_m, mo.ms()])
                if mo is b and len(mo.ops) > n_ops:
                    start = int(mo.ops[n_ops][2] * mo.mhz)
                    if start < busy_window_end:
                        events["errors"].append(f"operation started {(busy_window_end - start) / true_mhz:.1f} us "
                                                "before its BUSY window closed")
            mo.to_tick(local - 1)
            mo.call("bl_sess_poll", [sess_m, mo.ms()])
            mo.to_tick(local + int(120 * true_mhz))                     # decoded 100 us (the gap) + 20 us later
            mo.poke(sess_m + offs[FRAME_T0], struct.pack("<I", local & 0xffffffff))
            mo.poke(0x20006100, f)
            mo.bytes, mo.de = [], []
            mo.call("bl_sess_frame", [sess_m, 384, 0x20006100, mo.ms()])
            if mo.peek(sess_m + offs[REPLY], 1)[0]:
                mo.poke(sess_m + offs[REPLY], b"\0")
                if mo is obs:
                    events["errors"].append("the unselected bootloader answered")
                    continue
                ans = mo.peek(sess_m + offs[ANSWER], 32)
                at, brr = struct.unpack("<IH", mo.peek(sess_m + offs[REPLY_AT], 4) + mo.peek(sess_m + offs[BRR], 2))
                mo.poke(0x20006200, ans)
                mo.call("hw_reply", [0x20006200, at, brr, local & 0xffffffff], max_steps=2000000)
                if f[0] == 0x86:
                    events["polls"] += 1
                if mo.ret != 0:
                    events["silent_polls"] += 1
                    continue
                sent = [v for _, v in mo.bytes]
                assert sent == [0xff] + list(ans), "burst bytes"
                # timing on the host clock
                h = lambda tick: t_end + (tick - local) / true_mhz            # noqa: E731
                en = h(mo.de[0][1])
                first = h(mo.bytes[0][0])
                bit = mo.bit_ticks() / true_mhz
                last_stop = h(mo.bytes[-1][0]) + 10 * bit
                gaps = [h(mo.bytes[i + 1][0]) - (h(mo.bytes[i][0]) + 10 * bit) for i in range(len(mo.bytes) - 1)]
                off = h(mo.de[-1][1])
                baud_err = abs((1e6 / bit) / (1e6 / (1 << rate)) - 1)
                checks = [(270 <= en - t_end <= 900, f"enable at {en - t_end:.1f} us"),
                          (2 <= first - en <= 10, f"first start bit {first - en:.2f} us after enable"),
                          (max(gaps) <= 8, f"gap {max(gaps):.2f} us"),
                          (0 <= off - last_stop <= 5, f"DE low {off - last_stop:.2f} us after the stop bit"),
                          (off - t_end <= 900 + pitch - 30, f"burst end {off - t_end:.1f} us"),
                          (baud_err <= 0.015, f"baud error {baud_err * 100:.2f} %")]
                for ok, what in checks:
                    if not ok:
                        events["errors"].append(what)
                if events["first_answer_host"] is None:
                    events["first_answer_host"] = t_end
                events["answers"] += 1
                if ans[6 + 1] == 2:
                    events["busy"] += 1
                    busy_window_end = local + int((900 + pitch + 80) * true_mhz)
                if drop is not None and events["answers"] == drop:
                    continue                                             # this answer is lost on the wire
                r = lib.s_answer(bytes(ans), ctypes.c_longlong(int(off) + 1))
                if r != 0:
                    events["errors"].append(f"sender refused answer state {ans[7]} ({r})")
            elif f[0] == 0x86 and mo is b:
                events["polls"] += 1
                events["silent_polls"] += 1
            mo.call("bl_sess_poll", [sess_m, mo.ms()])
        events["frames"] += 1
        if b.peek(sess + offs[MODE], 1)[0] == 0:                         # BL_ACT_JUMP: leaving (the ACK)
            pass
    events["sender_state"] = lib.s_state()
    events["boot_complete"] = bool(lib.s_boot_complete())
    events["sender_error"] = lib.s_error()
    events["committed"] = b.peek(APP, 64) == new[:64] and b.peek(BODY, len(new) - 64) == new[64:]
    events["verified"] = b.peek(APP + 64, 2) == b"\xb7\xa5"
    events["exit_result"] = struct.unpack("<I", b.peek(sess + offs[EXIT_RESULT], 4))[0]
    events["leaving"] = b.peek(sess + offs[MODE], 1)[0] == 0
    events["erases"] = [hex(a) for k, a, _ in b.ops if k == "erase"]
    if obs:
        events["observer_untouched"] = bytes(obs.flash) == obs_before and not obs.ops
    events["entry"] = entry_seen
    if events["first_answer_host"] is not None and events["first_answer_host"] - first_frame_host < 1_000_000:
        events["errors"].append("an answer before 1 s of rate training")
    if len(set(events["erases"])) != len(events["erases"]):
        events["errors"].append("an erase repeated")
    return events


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("elf", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--proto", type=Path, default=Path(os.environ.get(
        "ULTRA_PROTOCOL_REPO", str(ROOT.parent / "quinled-ultra-protocol"))))
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    lib = build_sender(a.proto, a.out)
    lib.s_begin.argtypes = [ctypes.c_char_p, ctypes.c_uint, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint,
                            ctypes.c_uint, ctypes.c_uint, ctypes.c_longlong]
    lib.s_next.argtypes = [ctypes.c_longlong, ctypes.c_void_p]
    lib.s_answer.argtypes = [ctypes.c_char_p, ctypes.c_longlong]
    lib.s_due.restype = lib.s_poll_end.restype = ctypes.c_longlong
    offs = build_offsets(a.out)
    rows, bad = [], 0
    cases = [(0, 0.0, 772, None, True), (1, -0.045, 9000, None, False), (2, 0.022, 9000, 5, False),
             (0, 0.022, 5000, 3, True)]
    for rate, hsi, length, drop, observer in cases:
        ev = run(lib, a.elf, offs, a.out, rate, hsi, length, drop, observer)
        ok = (not ev["errors"] and ev["committed"] and ev["verified"] and ev["leaving"] and ev["exit_result"] == 1
              and ev["boot_complete"] and ev.get("observer_untouched", True) and ev["answers"] > 0)
        ev.update(rate=rate, hsi=hsi, length=length, drop=drop, ok=ok)
        rows.append(ev)
        bad += not ok
        print(json.dumps({k: ev[k] for k in ("rate", "hsi", "length", "drop", "ok", "answers", "busy",
                                              "silent_polls", "sender_state", "errors")}), flush=True)
    (a.out / "status_v407.json").write_text(json.dumps(rows, indent=2))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
