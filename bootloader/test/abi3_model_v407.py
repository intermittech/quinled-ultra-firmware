# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Peripheral model around the actual rv32emu production CH32V407 bootloader ELF (layout 4), after abi3_model.py.

The V407 bootloader speaks the update's status profile (docs/update.md): the session helpers here SELECT this MCU's
UID (ESIG, modeled), send the ANNOUNCE halves, poll (UPD_STATUS: the answer is read from the bootloader's answer
buffer; its wire timing is test_bl_status_v407.py's), run each operation a BUSY answer announced at its own time,
and send the blocks within credit. The rate lock comes from consistent trailer TIMEs (local ticks / 195.2).

The CH32V407 flash (RM V1.2 ch.32): erased halfword 0xE339 (bytes 39 E3), standard 4 KB sector erase (PER + ADDR +
STRT), fast 256-byte page program (MODEKEYR, FTPG, 64 words with WRBSY, PGSTRT), standard halfword program (PG), the
option bytes (OBKEYR, OBER erases all 128 bytes to 0xE339, OBPG writes a byte: the controller adds its complement).
Programming is allowed on erased cells only. Execution from the zero-wait flash goes on during an operation (RM p.579:
the erase / program code lives there); no data read may touch the range being written. Erase / program latency,
faults (no EOP, write protection, stuck BSY) and power cuts are modeled explicitly; a cut keeps only the flash and
option effects up to it, a restart constructs a new MCU (a software reset keeps the SRAM hand-over block).
"""
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import rv32emu                                                   # noqa: E402

rv32emu.FLASH_SIZE = 0x90000                                     # code flash up to the records (0x0808C000)
rv32emu.RAM_SIZE = 200 * 1024                                   # RAM_CODE_MOD = 0
from rv32emu import COST, Emu                                    # noqa: E402
from abi3_model import crc32, frame, reseal, seal                # noqa: E402,F401

FLASH, RCC, STK, CRC, PWR = 0x40022000, 0x40021000, 0xE000F000, 0x40023000, 0x40007000
KEYR, OBKEYR, STATR, CTLR, ADDR, OBR, WPR, MODEKEYR = (FLASH + o for o in (4, 8, 0x0C, 0x10, 0x14, 0x1C, 0x20, 0x24))
PG, PER, OBPG, OBER, STRT, LOCK, OBWRE, FLOCK, FTPG, PGSTRT = (1, 2, 0x10, 0x20, 0x40, 0x80, 0x200, 0x8000, 0x10000,
                                                               0x200000)
BSY, WRBSY, WRPRTERR, EOP = 1, 2, 0x10, 0x20
KEY1, KEY2 = 0x45670123, 0xCDEF89AB
OB = 0x1FFFF800
BOOT_END, APP, BODY, END = 0x08004000, 0x08004000, 0x08005000, 0x08080000
SECTOR, PAGE = 4096, 256
ERASED = b"\x39\xe3"
WORK, FRAME = 0x20006000, 0x20006100
UID = bytes(range(0x31, 0x3d))
ST = dict(frame_t0=0, reply=1, answer=2, op=3, op_due=4, op_at=5, committed=6, selected=7, locked=8, error=9, gate=10)
OFFSETS_C = r"""
#include "bl_core.h"
#include <stddef.h>
void b_init(void) { }
unsigned b_off(unsigned i)
{
    static const unsigned o[] = { offsetof(bl_sess_t, st.frame_t0), offsetof(bl_sess_t, st.reply),
        offsetof(bl_sess_t, st.answer), offsetof(bl_sess_t, st.op), offsetof(bl_sess_t, st.op_due),
        offsetof(bl_sess_t, st.op_at), offsetof(bl_sess_t, st.committed), offsetof(bl_sess_t, st.selected),
        offsetof(bl_sess_t, st.locked), offsetof(bl_sess_t, st.error), offsetof(bl_sess_t, gate) };
    return o[i];
}
"""
_OFFS = []


def offsets():
    """bl_sess_t.st field offsets, from the firmware's own header (compiled for RV32 once)"""
    if not _OFFS:
        import os
        import subprocess
        import tempfile
        gcc = Path(os.environ["LOCALAPPDATA"]) / "xPacks/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc.exe"
        with tempfile.TemporaryDirectory() as d:
            c, elf = Path(d) / "o.c", Path(d) / "o.elf"
            c.write_text(OFFSETS_C)
            subprocess.run([str(gcc), "-march=rv32imc_zicsr_zifencei", "-mabi=ilp32", "-Os", "-nostdlib", "-DCH32V407",
                            "-DBL_HW_ID=3", "-DMCU_CH32V40X=1", "-I", str(ROOT / "src/mcu/ch32v40x"),
                            "-I", str(ROOT / "src/core"), "-I", str(ROOT / "bootloader"),
                            "-Wl,-e,b_init", "-T", str(ROOT / "test/fx_rv.ld"), str(c), "-o", str(elf)],
                           check=True, timeout=120)
            e = Emu(str(elf))
            for i in range(len(ST)):
                e.call("b_off", [i])
                _OFFS.append(e.ret)
    return {k: _OFFS[v] for k, v in ST.items()}
HANDOVER = 32
PROVISIONED = [0xa5, 0xbf, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff]  # RDPR, USER (RAM_CODE_MOD 0), Data, WRPR0 = sectors 0-3


def erased(n):
    return ERASED * (n // 2)


def image(target, length=772, version=0x00030100, body=None):
    if body is None:
        body = bytes((i * 37 + i // 256 + 11) & 255 for i in range(length))
    length = len(body)
    desc = bytearray(64)
    desc[:5] = b"QDU6\x02"
    desc[6] = desc[7] = target[7]
    desc[8:15] = target[:7]
    desc[15] = 3
    struct.pack_into(">IIIII", desc, 16, version, length, crc32(body), BODY, BODY)
    desc[36:39] = bytes([2, 3, 1])
    return seal(desc) + body


class Reset(Exception):
    pass


class PowerCut(Exception):
    pass


class Booted(Exception):
    pass


class Recovery(Exception):
    pass


class AppMain(Exception):
    def __init__(self, registers, pc):
        super().__init__(hex(pc))
        self.registers, self.pc = registers, pc


class MCU(Emu):
    def __init__(self, elf, options=None, flash=None, handover=None):
        super().__init__(str(elf))
        self.elf = str(elf)
        boot = BOOT_END - 0x08000000
        if flash is None:
            self.flash[boot:] = erased(len(self.flash) - boot)
        else:
            self.flash[:] = flash
        if handover is not None:
            self.poke(0x20000000, handover)
        self.target = bytes(self.flash[21:37])
        self.now_us = 0.0
        self.timer = 0.0
        self.mhz = 195.2                         # (the session tests run at the app clock: clock_khz below)
        self.ops = []
        self.fail_at = self.cut_at = None
        self.fault = "eop"
        self.erase_us, self.program_us, self.half_us, self.ob_erase_us, self.ob_half_us = 30000, 2000, 100, 30000, 100
        self.cut_fraction = 0.5
        self.busy_us = None
        self.busy_range = None
        self.fast_buffer = []
        self.options = list(PROVISIONED if options is None else options)   # bytes; None = erased (0xE339)
        self.keys = {}
        self.stopping_boot = False
        self.app_main = self.app_entry = None
        self.app_mepc = 0
        self.app_stray = []
        self.uid = UID
        self.off = offsets()
        self.mmio[STK] = 0x25                            # SysTick running at HCLK (hw_full_init)
        self.load_options()
        self.mmio.update({CTLR: 0x02008080, STATR: 0, RCC + 0x24: 0x08000000, RCC: 0x83})
        self.setword("clock_khz", 195200)
        self.setword("options_trusted", 1, 1)

    # ------------------------------------------------------------------------------------------- option bytes
    def option_half(self, i):
        v = self.options[i]
        return 0xe339 if v is None else v | ((v ^ 255) << 8)

    def load_options(self):
        """OBR and WPR as loaded at reset (RM p.584-585): RAM_CODE_MOD = USER bit 6, OBERR on a bad complement"""
        halves = [self.option_half(i) for i in range(8)]
        bad = any(((h ^ (h >> 8)) & 255) != 255 for h in halves)
        user, wrp = halves[1] & 255, [halves[4 + k] & 255 for k in range(4)]
        self.mmio[OBR] = (((user >> 6) & 1) << 8) | (bad and 1 or 0) | ((halves[0] & 255) != 0xa5) << 1
        self.mmio[WPR] = wrp[0] | wrp[1] << 8 | wrp[2] << 16 | wrp[3] << 24

    def setword(self, symbol, value, size=4):
        self.poke(self.addr(symbol), value.to_bytes(size, "little"))

    def word(self, symbol):
        return int.from_bytes(self.peek(self.addr(symbol), 4), "little")

    def advance(self, us):
        assert us >= self.now_us
        self.timer += (us - self.now_us) * self.mhz
        self.now_us = float(us)

    def _check_app(self, pc, registers):
        if self.app_main is not None and pc in (self.app_main, self.app_main + 0x08000000):
            raise AppMain(list(registers), pc)

    def _i(self, instruction, pc, registers):
        self._check_app(pc, registers)
        if self.app_entry is not None:
            if instruction & 0x707f == 0x1073 and instruction >> 20 == 0x341:
                self.app_mepc = registers[(instruction >> 15) & 31]
                registers[(instruction >> 7) & 31] = 0
                return pc + 4, "alu"
            if instruction == 0x30200073:
                return self.app_mepc, "jump"
        self.last_pc = pc
        result = super()._i(instruction, pc, registers)
        self.advance(self.now_us + COST[result[1]] / self.mhz)
        return result

    def _c(self, instruction, pc, registers):
        self._check_app(pc, registers)
        self.last_pc = pc
        result = super()._c(instruction, pc, registers)
        self.advance(self.now_us + COST[result[1]] / self.mhz)
        return result

    def flash_addr(self, a):
        return a if 0x08000000 <= a < 0x08000000 + len(self.flash) else a + 0x08000000 if a < len(self.flash) else None

    def rd(self, a, n):
        if self.busy_us is not None and self.now_us < self.busy_us and self.busy_range:
            p = self.flash_addr(a)
            assert p is None or not (self.busy_range[0] <= p < self.busy_range[1]), \
                "a read of the range being written while BSY"
        if self.stopping_boot:
            if a in (BODY, BODY - 0x08000000):
                if self.app_main is None:
                    raise Booted()
                if self.app_entry is None:
                    self.app_entry = a
            if a == self.addr("bl_sess_init"):
                raise Recovery()
        if a == STK + 8:
            return int(self.timer) & 0xffffffff
        if a == STATR:
            if self.busy_us is not None:
                if self.busy_us == float("inf"):
                    self.advance(self.now_us + 100)
                    return self.mmio[STATR] | BSY
                self.advance(max(self.now_us, self.busy_us))
                self.busy_us = self.busy_range = None
            return self.mmio[STATR] & ~(BSY | WRBSY)
        if OB <= a < OB + 16:
            return self.option_half((a - OB) // 2)
        if 0x1FFFF7E8 <= a < 0x1FFFF7F4:
            return int.from_bytes(self.uid[a - 0x1FFFF7E8:a - 0x1FFFF7E8 + n], "little")
        if a == RCC:
            v = self.mmio.get(a, 0) | 2
            return v | (1 << 25 if v & (1 << 24) else 0)
        if a == RCC + 4:
            v = self.mmio.get(a, 0)
            return (v & ~12) | ((v & 3) << 2)
        return super().rd(a, n)

    def operation(self, kind, address, payload, latency):
        if kind in ("erase", "page", "half"):
            assert APP <= address and address + len(payload) <= END, (kind, hex(address), len(payload))
        if kind in ("page", "half"):
            old = self.peek(address, len(payload))
            assert old == erased(len(payload)), f"{kind} program on cells not erased at {address:#x}"
        if kind == "erase":
            assert address % SECTOR == 0, hex(address)
        self.ops.append((kind, address, self.now_us))
        count = len(self.ops)
        span = len(payload) if kind != "ob-half" else 2
        if count == self.cut_at:
            n = int(len(payload) * self.cut_fraction) & ~1
            self.apply(kind, address, payload[:n])
            raise PowerCut()
        if count == self.fail_at:
            self.mmio[STATR] = WRPRTERR if self.fault == "wrp" else 0
            if self.fault == "stuck":
                self.busy_us = float("inf")
            return
        self.apply(kind, address, payload)
        self.mmio[STATR] = EOP
        self.busy_us = self.now_us + latency
        self.busy_range = (address, address + span) if kind in ("erase", "page", "half") else None

    def apply(self, kind, address, payload):
        if kind == "ob-erase":
            if payload:
                self.options = [None] * 8
        elif kind == "ob-half":
            if payload:
                self.options[(address - OB) // 2] = payload[0]
        else:
            self.poke(address, payload)

    def wr(self, a, n, v):
        if self.app_entry is not None and not (0x20000000 <= a < 0x20000000 + rv32emu.RAM_SIZE or
                                               0x40000000 <= a < 0x60000000 or a >= 0xe0000000):
            self.app_stray.append(a)
        p = self.flash_addr(a)
        if p is not None and self.app_entry is None:
            assert APP <= p and p + n <= END, "CPU write outside the permitted metadata/body slot"
        if a == 0xe000e048 and v == 0xbeef0080:
            raise Reset()
        if a == CRC + 8:
            self.mmio[CRC] = 0xffffffff
        elif a == CRC:
            self.mmio[a] = crc32(v.to_bytes(4, "little"), self.mmio.get(a, 0xffffffff))
        elif a == STATR:
            self.mmio[a] &= ~(v & (EOP | WRPRTERR))
        elif a in (KEYR, OBKEYR, MODEKEYR):
            if self.keys.get(a) == KEY1 and v == KEY2:
                if a == KEYR:
                    self.mmio[CTLR] &= ~LOCK
                elif a == MODEKEYR:
                    self.mmio[CTLR] &= ~FLOCK
                elif not self.mmio[CTLR] & LOCK:
                    self.mmio[CTLR] |= OBWRE
            self.keys[a] = v
        elif a == CTLR:
            old = self.mmio[CTLR]
            v = (v & ~(LOCK | FLOCK | OBWRE)) | (old & (LOCK | FLOCK)) | (v & (LOCK | FLOCK)) | \
                (old & OBWRE & v)                                # LOCK / FLOCK set by software only, OBWRE cleared only
            if v & STRT:
                assert not old & LOCK, "start while locked"
                if v & PER:
                    self.operation("erase", self.mmio.get(ADDR, 0), erased(SECTOR), self.erase_us)
                elif v & OBER:
                    assert old & OBWRE, "option erase without OBWRE"
                    self.operation("ob-erase", OB, b"\0", self.ob_erase_us)
                v &= ~STRT
            if v & PGSTRT:
                assert v & FTPG and not old & (LOCK | FLOCK), "page start without the fast unlock"
                assert len(self.fast_buffer) == PAGE // 4, len(self.fast_buffer)
                base = self.fast_buffer[0][0]
                assert base % PAGE == 0 and [x for x, _ in self.fast_buffer] == list(range(base, base + PAGE, 4))
                self.operation("page", base, b"".join(d for _, d in self.fast_buffer), self.program_us)
                self.fast_buffer = []
                v &= ~PGSTRT
            self.mmio[a] = v
        elif p is not None:
            control = self.mmio[CTLR]
            assert not control & LOCK, "write while locked"
            if control & PG:
                assert n == 2
                self.operation("half", p, v.to_bytes(2, "little"), self.half_us)
            else:
                assert control & FTPG and not control & FLOCK and n == 4, "flash write outside a program mode"
                self.fast_buffer.append((p, v.to_bytes(4, "little")))
        elif OB <= a < OB + 16:
            control = self.mmio[CTLR]
            assert control & OBPG and control & OBWRE and n == 2, "option write outside OBPG"
            assert self.options[(a - OB) // 2] is None, "option byte programmed twice"
            self.operation("ob-half", a, bytes([v & 255]), self.ob_half_us)
        elif a == RCC + 4:
            self.mmio[a] = v
            self.mhz = 195.2 if v & 3 == 2 else 19.52
        elif a == STK and v & 32:
            self.mmio[a] = v
            self.timer = 0
        else:
            self.mmio[a] = v
            super().wr(a, n, v)

    # ------------------------------------------------------------------------------------------- helpers
    def installed(self, img, confirmed=True):
        meta = img[:64] + b"\xb7\xa5" + (b"\xc3\xa5" if confirmed else ERASED)
        self.poke(APP, meta + erased(SECTOR - len(meta)))
        self.poke(BODY, img[64:])

    def init_session(self):
        self.call("bl_app_read", [WORK])
        self.call("bl_sess_init", [self.addr("sess"), 3, WORK, int(self.now_us // 1000)])

    @property
    def state(self):
        return int.from_bytes(self.peek(self.addr("sess") + 4, 4), "little")

    # ------------------------------------------------------------------------------------------- status sessions
    def stv(self, name, n=4):
        return int.from_bytes(self.peek(self.addr("sess") + self.off[name], n), "little")

    def lock(self):
        """the rate lock as after >= 1 s of training (test_bl_status_v407.py trains for real): consistent TIMEs"""
        self.locked_from = int(self.timer)
        self.host0 = 5_000_000
        for k in range(12):                                # 1.1 s of keepalive PINGs, 100 ms apart
            self.send(frame(0x85, 0, 0, bytes(32)))
            self.advance(self.now_us + 100000)

    def host_us(self):
        return (self.host0 + int((self.timer - self.locked_from) / 195.2)) & 0xffffffff

    def ms(self):
        return int(self.timer / 195200)

    def send(self, f, time_us=None, run=True, corrupt=None):
        """a frame ending now: its TIME on the host clock, its edge at the current local tick"""
        if time_us is not None:
            self.advance(max(self.now_us, time_us))
        if not hasattr(self, "host0"):
            self.host0, self.locked_from = 5_000_000, int(self.timer)
        f = bytearray(reseal(f, self.host_us()))
        if corrupt is not None:
            f[corrupt] ^= 1                                # (after the stamp: the corruption stays)
        self.poke(self.addr("sess") + self.off["frame_t0"], (int(self.timer) & 0xffffffff).to_bytes(4, "little"))
        self.poke(FRAME, f)
        self.call("bl_sess_frame", [self.addr("sess"), 384, FRAME, self.ms()])
        self.advance(self.now_us + 300)                    # (the answer window, the next frame's spacing)
        if run:
            self.run_due()

    def run_due(self):
        """the main loop's bl_sess_poll: an announced operation at its op_at, the timeouts"""
        if self.stv("op_due", 1):
            at = self.stv("op_at")
            if (at - int(self.timer)) & 0x80000000 == 0:
                self.advance(at / self.mhz + 0.01)
        self.call("bl_sess_poll", [self.addr("sess"), self.ms()])
        return self.ret

    def poll(self, session=7, uid=None, run=True):
        """UPD_STATUS; the decoded answer (dict) or None (silence)"""
        data = (uid or self.uid) + bytes([0, 1, 0]) + bytes(17)
        self.poke(self.addr("sess") + self.off["reply"], b"\0")
        self.send(frame(0x86, session, 0, data), run=False)
        if not self.stv("reply", 1):
            if run:
                self.run_due()
            return None
        # The wire answer precedes the reserved blocking operation. A failure
        # there can cancel reply/op flags, but cannot retract bytes already sent.
        a = self.peek(self.addr("sess") + self.off["answer"], 32)
        v = a[6:30]
        result = dict(raw=a, version=v[0], state=v[1], error=v[2], credit=v[3], session=v[4], flags=v[5],
                    mask=int.from_bytes(v[6:8], "big"), accepted=int.from_bytes(v[8:12], "big"),
                    committed=int.from_bytes(v[12:16], "big"), retry=int.from_bytes(v[16:20], "big"),
                    crc=int.from_bytes(v[20:24], "big"))
        if run:
            self.run_due()
        return result

    def select(self, session=7, uid=None, target=None):
        self.send(frame(0x87, session, 0, (uid or self.uid) + (target or self.target) + bytes(4)))

    def drain(self, session=7, limit=400):
        """poll while BUSY (each BUSY announces one operation: it runs after the answer's window)"""
        for _ in range(limit):
            st = self.poll(session)
            if st is None or st["state"] != 2:
                return st
            self.advance(self.now_us + st["retry"] * 1000 + 600)
            self.run_due()
        raise AssertionError("BUSY without end")

    def start(self, img, session=7):
        self.select(session)
        self.send(frame(0x81, session, 0, img[:32]))
        self.send(frame(0x81, session, 1, img[32:64]))
        return self.drain(session)

    def transfer(self, img, session=7, skip=(), end=True):
        """blocks within credit (the chunks in skip never), END, the commit; the last status"""
        body = img[64:]
        n = (len(body) + 31) // 32
        st = self.poll(session)
        for _ in range(4 * n + 16):
            if st is None or st["state"] in (6, 7) or st["committed"] == len(body):
                break
            block = st["committed"] // 32
            sent = 0
            for k in range(8):
                index = block + k
                if index >= n or st["mask"] & (1 << k) or index in skip:
                    continue
                self.send(frame(0x82, session, index, body[index * 32:(index + 1) * 32].ljust(32, b"\0")))
                sent += 1
            st = self.drain(session)
            if not sent and st is not None and st["state"] not in (2,) and st["committed"] < len(body):
                break                                          # (a chunk is withheld: no progress)
        if end:
            self.send(frame(0x83, session, data=img[24:28] + img[20:24] + bytes(24)))
            st = self.drain(session)
        return st

    def ack(self, session=7):
        self.send(frame(0x85, session, 0, bytes(32)))

    def restarted(self, keep_handover=False):
        """a new MCU on this one's flash and option bytes (a software reset also keeps the SRAM hand-over block)"""
        return MCU(self.elf, self.options, bytes(self.flash), self.peek(0x20000000, HANDOVER) if keep_handover else None)

    def boot_path(self, app_main=None, max_resets=3):
        """main from reset at HSI: 'app' / 'bootloader' (or, with app_main, the AppMain of the installed app);
        a software reset (the protection write) boots again with the hand-over kept"""
        new = self.restarted()
        self.boot_chain = []
        for _ in range(max_resets + 1):
            self.boot_chain.append(new)
            new.setword("clock_khz", 20000)
            new.setword("options_trusted", 0, 1)
            new.mhz = 20.0
            new.stopping_boot = True
            new.app_main = app_main
            try:
                new.call("main", max_steps=3000000)
            except Booted:
                self.last_boot = new
                return "app"
            except Recovery:
                self.last_boot = new
                return "bootloader"
            except AppMain as reached:
                self.last_boot = new
                return new.app_entry, reached.registers, new.app_stray
            except Reset:
                new.mmio[RCC + 0x24] = 1 << 28          # (software reset flag)
                nxt = new.restarted(True)
                nxt.mmio[RCC + 0x24] = 1 << 28
                new = nxt
                continue
            raise AssertionError("no boot decision")
        raise AssertionError("reset loop")
