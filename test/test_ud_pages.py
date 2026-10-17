# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The QuinLED Diff-Power Ultra-Digital receiver's answers and status LED (BOARD=3) on the compiled firmware.

tools/rv32emu.py runs the ELF's own ud_frame (the header, the POLL, ud_rc_page, rc_answer, rc_brr) on frames built
here, and checks every answer (SYNC, PAGE, index, SEQ, UID16, CRC and each field) against the state poked into the ELF
and the protocol header's offsets (src/core/ultra_protocol.h, read here); then IDENTIFY 5 and an update's SAME / NACK
on the status LED (led_service). The state's field offsets come from test/ud_offsets.c, compiled here with the target
compiler ($RISCV_GCC, else the xPack GCC in %LOCALAPPDATA%\\xPacks) and read back from the object.

Build (the functions as written: no inlining or IPA clones, so they keep their arguments):
  XW=0 LTO=0 EXTRA_CFLAGS="-DBOARD=3 -fno-inline -fno-ipa-cp -fno-ipa-sra" OUT=build/ud-api bash tools/build_target.sh
Run: python test/test_ud_pages.py build/ud-api/fw.elf  (tools/build_release.sh runs it for UD=1)
"""
import glob
import os
import re
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / "tools"))
from rv32emu import Emu  # noqa: E402

checks = fails = 0


def check(ok, what):
    global checks, fails
    checks += 1
    if not ok:
        fails += 1
        print("FAIL:", what)


# ---------------------------------------------------------------------------------------------- the header's values
H = {}
HDR = (FW / "src/core/ultra_protocol.h").read_text(encoding="utf-8")
for line in HDR.splitlines():
    m = re.match(r"\s*#define\s+(ULTRA_\w+)\s+\(?(-?(?:0x[0-9A-Fa-f]+|\d+))u?\)?(?:\s|$)", line)
    if m:
        H[m.group(1)] = int(m.group(2), 0)
for m in re.finditer(r"\b(ULTRA_\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*[,}]", HDR):   # the enums' members
    H[m.group(1)] = int(m.group(2), 0)


def crc16(b):
    c = 0xFFFF
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021 if c & 0x8000 else c << 1) & 0xFFFF
    return c


def crc8(b):
    c = 0
    for x in b:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07 if c & 0x80 else c << 1) & 0xFF
    return c


def frame(slot_type, payload, seq, flags, host_us=123456):
    """one slot (light 0xF, the board) + the header: 40 bytes"""
    s = bytearray(H["ULTRA_SLOT_BYTES"])
    s[0], s[1], s[2] = slot_type, H["ULTRA_LIGHT_BOARD"] << 4, seq
    s[H["ULTRA_PAYLOAD"]:H["ULTRA_PAYLOAD"] + len(payload)] = payload
    s[30:32] = crc16(s[:30]).to_bytes(2, "big")
    h = bytearray(H["ULTRA_MAGIC"].to_bytes(2, "big") + bytes([H["ULTRA_VERSION"] << 4 | flags]) +
                  host_us.to_bytes(4, "big"))
    h.append(crc8(h))
    return bytes(s + h)


def poll(page, arg=b"", rate=0, seq=0x31, index=0, flags=None):
    p = bytes([index, 1, page, rate]) + arg     # slot bytes 3..: index, R, page, rate, arguments (byte 7 on)
    return frame(H["ULTRA_POLL"], p, seq, H["ULTRA_HDR_RW"] if flags is None else flags)


# ---------------------------------------------------------------------------------------------- the state's offsets
def target_gcc():
    if os.environ.get("RISCV_GCC"):
        return os.environ["RISCV_GCC"]
    xp = sorted(glob.glob(os.path.join(os.environ.get("LOCALAPPDATA", ""), "xPacks", "xpack-riscv-none-elf-gcc-*",
                                       "bin", "riscv-none-elf-gcc*")))
    if not xp:
        raise SystemExit("no target compiler: set RISCV_GCC")
    return xp[-1]


def offsets(elf):
    gen = Path(elf).parent / "gen"
    with tempfile.TemporaryDirectory() as td:
        obj = os.path.join(td, "ud_offsets.o")
        cmd = [target_gcc(), "-march=rv32imc_zicsr_zifencei", "-mabi=ilp32", "-Os", "-c", "-DCH32V20x_D6",
               "-DUPD_CRC_ULTRA", "-DBOARD=3", "-DMON_EXTRA=0", "-DCONSOLE_FULL=0", "-DUPPER_RECORDS=1",
               "-I" + str(FW / "vendor/wch"), "-I" + str(FW / "src/hal"), "-I" + str(FW / "src"), "-I" + str(gen),
               "-I" + str(FW / "src/core"),   # (the effect engine's header until src/core/fx.h exists)
               str(FW / "test/ud_offsets.c"), "-o", obj]
        subprocess.run(cmd, check=True)
        d = open(obj, "rb").read()
    shoff, = struct.unpack_from("<I", d, 32)
    shentsize, shnum, shstrndx = struct.unpack_from("<HHH", d, 46)
    secs = [struct.unpack_from("<IIIIIIIIII", d, shoff + i * shentsize) for i in range(shnum)]
    names = secs[shstrndx][4]
    for s in secs:
        nm = d[names + s[0]:d.index(b"\0", names + s[0])]
        if nm == b".rodata.ud_offs":
            v = struct.unpack_from("<%dI" % (s[5] // 4), d, s[4])
            assert v[0] == 0x55443300 and v[-1] == 0x55443399, "ud_offsets.c's table"
            keys = ("en_lt on_s lit_s ups trips dips peak_t peak_a wd updates writes upd_info info_hw_id info_hw_rev "
                    "info_bl vs_valid vs_mv mt_c10 mt_vdda dl_st st_frames st_bits_last st_bits_min st_bits_max "
                    "st_long_high st_answered st_marks st_cap_drops dl_wpos dl_edge_w dl_direct uds_fps10 uds_faults "
                    "uds_polls uds_headers uds_tail_err uds_decoded uds_dec_err dl_cap upd_frames "
                    "ux_cfg ux_npx ux_chip chip_bpp ux_seg seg_run seg_p fx_effect fx_bri ux_want_local ux_want_stream "
                    "ux_stored_have ux_save_due ux_seq5 ux_lock ux_a_have ux_a_host ux_local ux_dark ux_save_ms "
                    "dl_ch_on dl_ch_st dl_ch_end dl_ch_tx dl_ch_win dl_ch_bits dl_ch_hn dl_ch_tn dl_ch_tw dl_ch_arm dl_ch_ready "
                    "dl_ch_enter dl_ch_hold dl_ch_req uc_mode uc_cfg uc_flags uc_slots uc_frames uc_nonultra uc_longs "
                    "uc_windows uc_missed uc_markers uc_bits ux_chain ux_wpu24 ux_r_have "
                    "sz_seg sz_life sz_info").split()
            assert len(keys) == len(v) - 2
            return dict(zip(keys, v[1:-1]))
    raise SystemExit("ud_offsets.o: no .rodata.ud_offs")


def every_symbol(elf, name):
    """addresses of every symbol called name (local statics of several files share names)"""
    d = open(elf, "rb").read()
    shoff, = struct.unpack_from("<I", d, 32)
    shentsize, shnum = struct.unpack_from("<HH", d, 46)
    secs = [struct.unpack_from("<IIIIIIIIII", d, shoff + i * shentsize) for i in range(shnum)]
    out = []
    for s in secs:
        if s[1] == 2:
            strtab = secs[s[6]][4]
            for k in range(s[5] // 16):
                st_name, st_value = struct.unpack_from("<II", d, s[4] + k * 16)
                if d[strtab + st_name:d.index(b"\0", strtab + st_name)] == name.encode():
                    out.append(st_value)
    return out


# ---------------------------------------------------------------------------------------------- the board's state
elf = sys.argv[1] if len(sys.argv) > 1 else str(FW / "build/ud-api/fw.elf")
O = offsets(elf)
e = Emu(elf)
A = e.addr
w = e.wr

FW_WORD = 0x000701FF                           # 0.7.1, a release (pre-release 0xFF)
UP = (1 << 24) + 5                             # STATUS's u24 uptime wraps
UID = bytes(range(0x11, 0x1D))
LT = dict(on_s=3600 * 1234 + 359, lit_s=3600 * 456 + 1, ups=70000, dips=0x123456, writes=0x00ABCDEF)
w(A("fw_version"), 4, FW_WORD)
w(A("up_s"), 4, UP)
for a in every_symbol(elf, "reset_flags"):
    w(a, 4, 0x0C000000)                        # power-on + pin: STATUS 7 = 0x03
upd = A("upd") + O["upd_info"]
w(upd + O["info_hw_id"], 1, 3)
w(upd + O["info_hw_rev"], 1, 0)
w(upd + O["info_bl"], 1, 2)
lt = A("en") + O["en_lt"]
for k, v in LT.items():
    w(lt + O[k], 4, v)
w(lt + O["trips"], 2, 3)
w(lt + O["trips"] + 2, 2, 65535)
w(lt + O["updates"], 2, 2)
w(lt + O["wd"], 1, 255)
w(lt + O["peak_t"], 1, 150)
w(lt + O["peak_a"], 1, 42)
w(A("vs_bus") + O["vs_valid"], 1, 1)
w(A("vs_bus") + O["vs_mv"], 4, 24123)
w(A("mt") + O["mt_c10"], 2, (-125) & 0xFFFF)  # -12.5 C
w(A("mt") + O["mt_vdda"], 2, 3301)
w(A("i_avg"), 4, 1234)
w(A("rc_uid16"), 2, 0xBEEF)
st = A("dl") + O["dl_st"]
w(st + O["st_frames"], 4, 0x01234567)
w(st + O["st_bits_last"], 4, 14400)
w(st + O["st_bits_min"], 4, 0xFFFFFFFF)
w(st + O["st_bits_max"], 4, 65535)
w(st + O["st_long_high"], 2, 3)
w(st + O["st_answered"], 2, 7)
w(st + O["st_marks"], 2, 11)
w(st + O["st_cap_drops"], 1, 2)
w(A("dl") + O["dl_wpos"], 4, 2250 * 5 + 7)  # 5 ms since the last edge
w(A("dl") + O["dl_edge_w"], 4, 7)
w(A("dl") + O["dl_direct"], 1, 0)
uds = A("uds")
w(uds + O["uds_fps10"], 2, 600)
w(uds + O["uds_faults"], 1, 2)
w(uds + O["uds_polls"], 2, 25)
for i, b in enumerate(UID):
    e.mmio[0x1FFFF7E8 + i] = b

FBUF, ANS = A("ud_fbuf"), A("ud_ans")
tick = [1000]


def run(f, nbits=320):
    e.poke(FBUF, f)
    e.poke(ANS, bytes(32))
    tick[0] += 20 * 50                          # 50 ms of 20 kHz ticks between frames
    e.call("ud_frame", [FBUF, nbits, tick[0]])
    return e.ret, e.peek(ANS, 32)


def u(b, o, n, signed=False):
    return int.from_bytes(b[o:o + n], "big", signed=signed)


def answer(page, arg=b"", rate=0, expect=None, seq=0x31):
    """a POLL answered: the framing checked; returns DATA"""
    what = "page 0x%02X arg %s rate %d" % (page, arg[:1].hex() or "-", rate)
    r, a = run(poll(page, arg, rate, seq))
    check(r == 1, what + ": ud_frame %r (1 = an answer armed)" % r)
    check(a[0] == H["ULTRA_RSP_SYNC"] and u(a, 30, 2) == crc16(a[:30]), what + ": SYNC / CRC")
    check(a[1] == (page if expect is None else expect), what + ": answered page 0x%02X" % a[1])
    check(a[2] >> 4 == 0 and a[3] == seq and u(a, 4, 2) == 0xBEEF, what + ": index / SEQ / UID16")
    return a[H["ULTRA_RSP_DATA"]:H["ULTRA_RSP_DATA"] + H["ULTRA_RSP_DATA_LEN"]], e.rd(A("ud_brr"), 2)


# ---------------------------------------------------------------------------------------------- the pages
FEATS = sum(H[n] for n in ("ULTRA_FEAT_MCU_TEMP", "ULTRA_FEAT_VERSION_BLINK", "ULTRA_FEAT_POWER_EVENTS",
                           "ULTRA_FEAT_SUPPLY_DIPS", "ULTRA_FEAT_ENERGY", "ULTRA_FEAT_LIFETIME", "ULTRA_FEAT_DIAG",
                           "ULTRA_FEAT_STANDBY", "ULTRA_FEAT_LIFE_SYNC"))   # (no FW_UPDATE: a standalone build)
brr = []
for rate in range(3):
    d, b = answer(H["ULTRA_PAGE_IDENTITY"], bytes([H["ULTRA_ID_PART_BOARD"]]), rate)
    brr.append(b)
    check(d[0:12] == UID, "IDENTITY 0: the UID")
    check(u(d, H["ULTRA_ID_FW_VERSION"], 4) == FW_WORD, "IDENTITY 0: the firmware word")
    check(d[H["ULTRA_ID_BL_VERSION"]] == 2 and d[H["ULTRA_ID_HW_ID"]] == 3 and d[H["ULTRA_ID_HW_REV"]] == 0,
          "IDENTITY 0: bootloader / hardware id / revision")
    check(d[19] == 0 and u(d, H["ULTRA_ID_CAPS"], 4) == 0x1CD and u(d, H["ULTRA_ID_CAPS"], 4) & H["ULTRA_HWCAP_CHAIN"] and
          not u(d, H["ULTRA_ID_CAPS"], 4) & H["ULTRA_HWCAP_RELAY"], "IDENTITY 0: capabilities 0x%X (CHAIN, no RELAY)" % u(d, 20, 4))
    d, _ = answer(H["ULTRA_PAGE_IDENTITY"], bytes([H["ULTRA_ID_PART_FEATURES"]]), rate)
    check(u(d, H["ULTRA_IDF_FEATURES"], 4) == FEATS and d[4:24] == bytes(20),
          "IDENTITY 1: features 0x%08X (want 0x%08X), words 1-5 zero" % (u(d, 0, 4), FEATS))
    answer(H["ULTRA_PAGE_IDENTITY"], bytes([2]), rate, expect=H["ULTRA_PAGE_UNSUPPORTED"])
    d, _ = answer(H["ULTRA_PAGE_STATUS"], b"", rate)
    check(u(d, H["ULTRA_ST_FW_VERSION"], 3) == FW_WORD >> 8, "STATUS: firmware")
    check(d[H["ULTRA_ST_HW_ID"]] == 3, "STATUS 3: the hardware id %d" % d[3])
    check(u(d, H["ULTRA_ST_UPTIME"], 3) == UP & 0xFFFFFF, "STATUS: uptime (u24)")
    check(d[H["ULTRA_ST_RESET"]] == 0x03 and d[H["ULTRA_ST_STATE"]] == 0, "STATUS: reset / state")
    check(u(d, H["ULTRA_ST_MCU_TEMP"], 2, True) == -125 and u(d, H["ULTRA_ST_TEMP_BOARD"], 2) == H["ULTRA_ST_NA"],
          "STATUS: MCU / board temperature")
    check(u(d, H["ULTRA_ST_VBUS"], 2) == 2412 and u(d, H["ULTRA_ST_VDDA"], 2) == 3301 and
          u(d, H["ULTRA_ST_CURRENT"], 2) == 1234, "STATUS: bus / VDDA / current")
    check(d[H["ULTRA_ST_MEASURED"]] == H["ULTRA_ST_M_VBUS"] | H["ULTRA_ST_M_CURRENT"] and
          d[H["ULTRA_ST_PWM_PRESET"]] == 0xFF and u(d, H["ULTRA_ST_CONFIG_CRC"], 2) == 0xFFFF and
          d[H["ULTRA_ST_MODES"]] == 0, "STATUS: measured / PWM / CONFIG CRC / modes")
check(0 < brr[0] < brr[1] < brr[2], "USART3's divider per rate: %s" % brr)

d, _ = answer(H["ULTRA_PAGE_LIFETIME"])
check(u(d, H["ULTRA_LT_POWERED"], 3) == LT["on_s"] // 360 and u(d, H["ULTRA_LT_LIT"], 3) == LT["lit_s"] // 360,
      "LIFETIME: hours")
check(u(d, H["ULTRA_LT_POWER_UPS"], 3) == 70000 and u(d, H["ULTRA_LT_DIPS"], 3) == 0x123456 and
      u(d, H["ULTRA_LT_RECORDS"], 3) == 0xABCDEF, "LIFETIME: power-ups / dips / records")
check(u(d, H["ULTRA_LT_OVERLOAD_TRIPS"], 2) == 3 and u(d, H["ULTRA_LT_SHORT_TRIPS"], 2) == 65535 and
      d[H["ULTRA_LT_WATCHDOG"]] == 255 and u(d, H["ULTRA_LT_UPDATES"], 2) == 2, "LIFETIME: trips / watchdog / updates")
check(d[H["ULTRA_LT_TEMP_PEAK"]] == 150 and d[H["ULTRA_LT_CURRENT_PEAK"]] == 42, "LIFETIME: peaks")

d, _ = answer(H["ULTRA_PAGE_DIAG"], bytes([0]))
check(d[H["ULTRA_DG_RESET"]] == 0x03 and d[H["ULTRA_DG_C10_LOAD"]] == 0xFF and d[H["ULTRA_DG_C10_PEAK"]] == 0xFF,
      "DIAG 0: reset / no C10 load")
d, _ = answer(H["ULTRA_PAGE_DIAG"], bytes([1]))
check(all(u(d, H["ULTRA_DG_TREND"] + i, 1, True) == H["ULTRA_HE_NA"] for i in range(5)) and
      u(d, H["ULTRA_DG_TREND_LIT_H"], 2) == 0xFFFF, "DIAG 1: no calibration trend")
answer(H["ULTRA_PAGE_DIAG"], bytes([2]), expect=H["ULTRA_PAGE_UNSUPPORTED"])

d, _ = answer(H["ULTRA_PAGE_ENERGY"], b"", 1)
check(u(d, H["ULTRA_EN_POWER_MW"], 4) == 24123 // 10 * 1234 // 100, "ENERGY: power %d mW" % u(d, 8, 4))
check(u(d, H["ULTRA_EN_VB"], 2) == 0xFFFF, "ENERGY: V_B not measured")

pre = {k: e.rd(uds + O[k], 2) for k in ("uds_headers", "uds_tail_err", "uds_decoded", "uds_dec_err", "uds_polls")}
d, _ = answer(H["ULTRA_PAGE_ERRORS"])
now = {k: e.rd(uds + O[k], 2) for k in pre}     # (the page shows the counts after this frame's own)
check(u(d, H["ULTRA_ER_FRAMES"], 2) == 0x4567 and u(d, H["ULTRA_ER_HEADERS"], 2) == now["uds_headers"] and
      u(d, H["ULTRA_ER_TAIL_ERRORS"], 2) == now["uds_tail_err"] and u(d, H["ULTRA_ER_DECODED"], 2) == now["uds_decoded"]
      and u(d, H["ULTRA_ER_DECODE_ERRORS"], 2) == now["uds_dec_err"], "ERRORS 0-8: frames / headers / decoded")
check(u(d, H["ULTRA_ER_WINDOWS"], 2) == 11 and u(d, H["ULTRA_ER_ANSWERS"], 2) == 7 and
      u(d, H["ULTRA_ER_MISSED"], 2) == now["uds_polls"] - 7 and u(d, H["ULTRA_ER_ABORTED"], 2) == 2,
      "ERRORS 10-16: marker frames / answered / not answered / not captured")
check(now["uds_headers"] == pre["uds_headers"] + 1 and now["uds_polls"] == pre["uds_polls"] + 1, "ERRORS: counted")

for page in (H["ULTRA_PAGE_POWER"], H["ULTRA_PAGE_SUPPLY"]):
    answer(page, bytes(9))
for page in (0x01, 0x02, 0x05, 0x06, 0x07, H["ULTRA_PAGE_HEALTH"], 0x0F, 0x7F, 0x82):
    answer(page, expect=H["ULTRA_PAGE_UNSUPPORTED"])

d, _ = answer(0x80, b"", 2)                     # PIXEL (main_ud.c UD_PX_*)
check(u(d, 0, 2) == 600 and u(d, 2, 3) == 14400 and u(d, 5, 3) == 0xFFFFFF and u(d, 8, 3) == 65535,
      "PIXEL: fps x 10 / bits last / min none / max")
check(u(d, 11, 3) == 0x234567 and u(d, 14, 2) == 3 and u(d, 16, 2) == 5, "PIXEL: frames u24 / long HIGHs / ms")
check(d[18] == 1 and d[19] == 2 and u(d, 20, 2) == e.rd(uds + O["uds_polls"], 2) and u(d, 22, 2) == 7,
      "PIXEL: mode delayed / faults / polls / answered")
check(e.rd(st + O["st_bits_min"], 4) == 0xFFFFFFFF and e.rd(st + O["st_bits_max"], 4) == 0, "PIXEL: min / max restart")

# PIXEL_FX (0x81, three parts): the state (the effect engine's counts are its own), the effect and palette maps
d, _ = answer(H["ULTRA_PAGE_HW_FX"], bytes([0]), 0)
check(u(d, H["ULTRA_PXG_CATALOGUE"], 2) == 3 and d[H["ULTRA_PXG_CHIPS"]] == H["ULTRA_PXC_COUNT"] and
      d[H["ULTRA_PXG_SEGMENTS"]] == H["ULTRA_PX_SEGMENTS"] and d[H["ULTRA_PXG_MODE"]] == H["ULTRA_PXM_STREAM"] and
      d[H["ULTRA_PXG_EFFECTS"]] >= 1 and d[H["ULTRA_PXG_PALETTES"]] >= 1, "PIXEL_FX 0: catalogue / chips / segments / stream mode")
for part in (1, 2):
    d, _ = answer(H["ULTRA_PAGE_HW_FX"], bytes([part]), 0)
    check(any(d[:24]), "PIXEL_FX %d: the map has bits" % part)
answer(H["ULTRA_PAGE_HW_FX"], bytes([4]), 0, expect=H["ULTRA_PAGE_UNSUPPORTED"])
d, _ = answer(H["ULTRA_PAGE_HW_FX"], bytes([H["ULTRA_PXG_PART_LINE"]]), 0)      # part 3: the line (AUTO, deciding, nothing seen yet)
check(d[H["ULTRA_PXL_CONFIG"]] == 0 and d[H["ULTRA_PXL_MODE"]] == 0 and d[H["ULTRA_PXL_FLAGS"]] == 0 and
      u(d, H["ULTRA_PXL_FRAMES"], 2) == 0 and u(d, H["ULTRA_PXL_WINDOWS"], 2) == 0 and u(d, H["ULTRA_PXL_ANSWERS"], 2) == 7,
      "PIXEL_FX 3: the line: AUTO, undecided, nothing counted; answers = dline's %d" % u(d, H["ULTRA_PXL_ANSWERS"], 2))

# the local effects' commands through ud_frame on the compiled firmware: CONFIG 129-139, PIXEL_FX, the anchor
ux = A("ux")
e.call("udfx_init", [ux])
cfg = bytearray()
for cid, val in ((129, 3), (131, 40), (138, 5), (140, 9), (132, 0), (0, 0), (130, 0), (139, 50), (133, 150)):
    cfg += bytes([cid]) + val.to_bytes(2, "big")
r, _ = run(frame(H["ULTRA_CONFIG"], bytes(cfg), 9, 0))
vals = [e.rd(ux + O["ux_cfg"] + 2 * k, 2) for k in range(13)]
check(r == 0 and vals == [3, 0, 40, 0, 150, 0, 0, 0, 0, 5, 50, 0, 0], "CONFIG 129-141 through ud_frame (140 = 9 refused): %s" % vals)
check(e.rd(ux + O["ux_save_due"], 1) == 1, "CONFIG: the record is due")
e.call("udfx_service", [ux, A("dl"), 5, 0, 256])
check(e.rd(ux + O["ux_npx"], 2) == 40 and e.rd(ux + O["ux_chip"] + O["chip_bpp"], 1) == 4, "the chip / pixel count derived")
fx = bytearray(H["ULTRA_PX_CUSTOM2"] + 1 - H["ULTRA_PAYLOAD"])
fx[0] = 3                                       # an effect (the engine's id 3)
fx[H["ULTRA_PX_BRIGHTNESS"] - H["ULTRA_PAYLOAD"]] = 200
fx[H["ULTRA_PX_FLAGS"] - H["ULTRA_PAYLOAD"]] = H["ULTRA_PXF_STORE"]
r, _ = run(frame(H["ULTRA_PIXEL_FX"], bytes(fx), 4, 0))
seg0 = ux + O["ux_seg"]
check(e.rd(seg0 + O["seg_run"], 1) == 0, "PIXEL_FX waits for a render boundary")
# This command-only test explicitly advances frame setup; the wire/tick suites
# exercise the real latch-safe entry and publication.
e.wr(ux + O["ux_local"], 1, 1)
for _ in range(10):
    e.call("udfx_render", [ux])
e.wr(ux + O["ux_local"], 1, 0)
check(r == 0 and e.rd(seg0 + O["seg_run"], 1) == 1 and e.rd(seg0 + O["seg_p"] + O["fx_effect"], 1) == 3 and
      e.rd(seg0 + O["seg_p"] + O["fx_bri"], 1) == 200, "PIXEL_FX through ud_frame: the effect runs")
check(e.rd(ux + O["ux_want_local"], 1) == 0 and e.rd(ux + O["ux_stored_have"], 1) == 1 and e.rd(ux + O["ux_seq5"] + 4, 1) == 4,
      "PIXEL_FX: stored, SEQ taken (LOCAL is asked for by the service, not by the command)")
e.call("udfx_service", [ux, A("dl"), 6, 0, 256])
check(e.rd(ux + O["ux_want_local"], 1) == 1, "the service asks for LOCAL once an effect runs")
check(e.rd(ux + O["ux_a_have"], 1) == 1, "the marker frame anchored the host clock")
fx[0] = 7
r, _ = run(frame(H["ULTRA_PIXEL_FX"], bytes(fx), 4, 0))   # the same SEQ: ignored
check(e.rd(seg0 + O["seg_p"] + O["fx_effect"], 1) == 3, "PIXEL_FX: the same SEQ applied again")
fx[0] = 0
r, _ = run(frame(H["ULTRA_PIXEL_FX"], bytes(fx), 5, 0))   # effect 0 + STORE: back to the stream, the stored effect cleared
e.call("udfx_service", [ux, A("dl"), 7, 0, 256])       # already STREAM: consume cancellation without a render
check(e.rd(seg0 + O["seg_run"], 1) == 0 and e.rd(ux + O["ux_want_stream"], 1) == 1 and e.rd(ux + O["ux_stored_have"], 1) == 0,
      "PIXEL_FX 0 + STORE: stream, cleared")
r, _ = run(frame(H["ULTRA_PIXEL_FX"], bytes(fx), 6, H["ULTRA_HDR_EXT"]))   # an EXT frame: every slot ignored
check(e.rd(ux + O["ux_seq5"] + 4, 1) == 5, "PIXEL_FX in an EXT frame was taken")

# LIFE_SYNC: the ledger 2 h and 5 power-ups ahead; a wrong UID16, a wrong RECORDS byte, the merge, then once only
lt16 = bytearray(16)
lt16[0:3] = (LT["on_s"] // 360 + 20).to_bytes(3, "big")
lt16[3:6] = (LT["lit_s"] // 360).to_bytes(3, "big")
lt16[6:9] = (70005).to_bytes(3, "big")
lt16[9:11] = (3).to_bytes(2, "big")
lt16[11:13] = (65535).to_bytes(2, "big")
lt16[13:16] = (0x123456).to_bytes(3, "big")
for uid16, rec, want, flags in ((0x1234, 0xEF, "ULTRA_LS_R_UID", 0), (0xBEEF, 0x00, "ULTRA_LS_R_RECORDS", 0),
                                (0xBEEF, 0xEF, "ULTRA_LS_R_APPLIED", H["ULTRA_LS_F_POWERED"] | H["ULTRA_LS_F_UPS"]),
                                (0xBEEF, 0xEF, "ULTRA_LS_R_ONCE", 0)):
    d, _ = answer(H["ULTRA_PAGE_LIFE_SYNC"], uid16.to_bytes(2, "big") + bytes(lt16) + bytes(4) + bytes([rec]))
    check(d[H["ULTRA_LS_A_RESULT"]] == H[want] and d[H["ULTRA_LS_A_APPLIED"]] == flags,
          "LIFE_SYNC %s: result %d applied 0x%02X" % (want, d[20], d[21]))
check(e.rd(lt + O["ups"], 4) == 70005 and e.rd(lt + O["on_s"], 4) // 360 == LT["on_s"] // 360 + 20,
      "LIFE_SYNC: merged into the record")

# not for this board: RELAY_ONLY, another index, a frame without RW, a bad slot CRC, another length
r, _ = run(poll(H["ULTRA_PAGE_STATUS"], index=H["ULTRA_POLL_RELAY_ONLY"]))
check(r == 0, "RELAY_ONLY: no answer")
r, _ = run(poll(H["ULTRA_PAGE_STATUS"], index=1))
check(r == 0, "index 1: no answer (one receiver per port)")
r, _ = run(poll(H["ULTRA_PAGE_STATUS"], flags=0))
check(r == 0, "a POLL slot without RW: no answer")
f = bytearray(poll(H["ULTRA_PAGE_STATUS"]))
f[10] ^= 1
derr = e.rd(uds + O["uds_dec_err"], 2)
r, _ = run(bytes(f))
check(r == 0 and e.rd(uds + O["uds_dec_err"], 2) == derr + 1, "a bad slot CRC: no answer, a decode error")


# the capture path (ud_decode): the bits counted to the frame's real end, exactly 320 or 384
def wave(data, nbits):
    s = [0] * 360                               # 10 us LOW after the marker's falling edge
    for i in range(nbits):
        hi = 25 if data[i // 8] & (0x80 >> (i % 8)) else 9
        s += [1] * hi + [0] * (48 - hi)
    s += [0] * 1664                             # the frame end's 46 us LOW
    s += [0] * (-len(s) % 16)
    return [int("".join(map(str, s[k:k + 16])), 2) for k in range(0, len(s), 16)]


def decode(data, nbits):
    words = wave(data, nbits)
    cap = A("dl") + O["dl_cap"]
    e.poke(cap, struct.pack("<%dH" % len(words), *words))
    tick[0] += 20 * 50
    e.call("ud_decode", [cap, len(words), tick[0]], max_steps=5_000_000)
    return e.ret


blk = bytearray(40)                             # an UPD PING: the 40-byte block (ultra_update.h) + its header
blk[0] = 0x85
blk[38:40] = crc16(blk[:38]).to_bytes(2, "big")
hdr = bytearray(H["ULTRA_MAGIC"].to_bytes(2, "big") + bytes([2 << 4 | H["ULTRA_HDR_UPD"]]) + (777).to_bytes(4, "big"))
hdr.append(crc8(hdr))
upd_f = bytes(blk) + bytes(hdr)
n0 = e.rd(A("upd") + O["upd_frames"], 4)
check(decode(poll(H["ULTRA_PAGE_STATUS"]), 320) == 1, "ud_decode: a 320-bit POLL answered")
check(decode(upd_f, 384) == 0 and e.rd(A("upd") + O["upd_frames"], 4) == n0 + 1, "ud_decode: a 384-bit UPD frame taken")
for n, f in ((321, poll(H["ULTRA_PAGE_STATUS"]) + bytes(8)), (328, poll(H["ULTRA_PAGE_STATUS"]) + bytes(8)),
             (383, upd_f), (385, upd_f + bytes(1))):
    derr = e.rd(uds + O["uds_dec_err"], 2)
    check(decode(f, n) == 0 and e.rd(uds + O["uds_dec_err"], 2) == derr + 1 and
          e.rd(A("upd") + O["upd_frames"], 4) == n0 + 1, "ud_decode: %d bits refused" % n)


# ---------------------------------------------------------------------------------------------- CHAIN frames (ud_chain_decode)
def chain_words(data, phase=5, js=123457):
    """a chain frame as dline's chain scanner keeps it (core/dline.c dl_ch_scan): the head (the first 752 words, the first one LOW) and
    the snapshot (every word of the frame, up to the one that ends it, at its own ring position js + i, mod 512)"""
    s = [0] * phase
    for byte in data:
        for k in range(8):
            hi = 25 if byte & (0x80 >> k) else 12
            s += [1] * hi + [0] * (45 - hi)
    s += [0] * (-len(s) % 16)
    words = [int("".join(map(str, s[k:k + 16])), 2) for k in range(0, len(s), 16)]
    nz = max(i for i, v in enumerate(words) if v)
    frame = words[:nz + 1] + [0] * 104
    head = [0] + frame[:751]
    snap = [0] * 512
    for i, v in enumerate(frame):
        snap[(js + i) & 511] = v
    return head, snap, len(head), len(frame), js + len(frame)


DL = A("dl")
UC = A("uc")


def poll_r(index, r, page, rate, seq):
    """a POLL for answer index `index` of r, in an RW frame"""
    return frame(H["ULTRA_POLL"], bytes([index, r, page, rate]), seq, H["ULTRA_HDR_RW"])

chain_n = [0]
decode_cycles = []


def chain_decode(data, bits, end_w, phase=5):
    head, snap, hn, tn, tw = chain_words(data, phase)
    cap = DL + O["dl_cap"]
    e.poke(cap, struct.pack("<%dH" % hn, *head))
    e.poke(cap + 2 * 752, struct.pack("<512H", *snap))
    w(DL + O["dl_ch_bits"], 2, bits)
    w(DL + O["dl_ch_hn"], 2, hn)
    w(DL + O["dl_ch_tn"], 2, tn - 96)                  # (the header's window ends 8 LOW words after the last pulse: 96 words before the snapshot's end)
    w(DL + O["dl_ch_tw"], 4, tw - 96)
    w(DL + O["dl_ch_end"], 4, end_w)
    w(DL + O["dl_ch_ready"], 1, 1)
    w(A("dl_ch_ready"), 1, 1)
    w(A("dl_ch_tick"), 4, 1000 + 20 * 50 * chain_n[0])
    chain_n[0] += 1
    e.poke(ANS, bytes(32))
    e.call("ud_chain_decode", [], max_steps=8_000_000)
    decode_cycles.append(e.cycles())


def ucv(k, n=2):
    return e.rd(UC + O[k], n)


e.call("udfx_init", [ux])
e.poke(UC, bytes(e.sym("uc")[1]))               # the line mode state zero: AUTO, undecided
e.poke(DL + O["dl_ch_req"], bytes(1))
h0 = e.rd(uds + O["uds_headers"], 2)
d0 = e.rd(uds + O["uds_decoded"], 2)
t0 = e.rd(uds + O["uds_tail_err"], 2)
nop = frame(H["ULTRA_NOP"], b"", 1, 0, 10000)
chain_decode(nop, 320, 120000)
check(ucv("uc_frames") == 1 and ucv("uc_mode", 1) == 0 and e.rd(DL + O["dl_ch_ready"], 1) == 0 and e.rd(DL + O["dl_ch_arm"], 1) == 0 and
      e.rd(uds + O["uds_headers"], 2) == h0 + 1 and e.rd(uds + O["uds_decoded"], 2) == d0 + 1,
      "CHAIN: a NOP frame: counted, undecided after one, the capture released, no answer")
check(e.rd(DL + O["dl_ch_req"], 1) == 0 and e.rd(ux + O["ux_chain"], 1) == 0, "CHAIN: nothing started by one frame")
chain_decode(nop, 320, 130000)
check(ucv("uc_mode", 1) == 2 and e.rd(DL + O["dl_ch_req"], 1) == 5 and e.rd(ux + O["ux_chain"], 1) == 1,
      "CHAIN: the second Ultra frame decides CHAIN (dline_ch_start enter, the encoder asked for): mode %d req %d" %
      (ucv("uc_mode", 1), e.rd(DL + O["dl_ch_req"], 1)))
chain_decode(frame(H["ULTRA_NOP"], b"", 1, 0, 10040), 1000, 140000)
check(ucv("uc_nonultra") == 1 and e.rd(uds + O["uds_tail_err"], 2) == t0 + 1 and e.rd(DL + O["dl_ch_arm"], 1) == 0 and ucv("uc_mode", 1) == 2,
      "CHAIN: 1000 bits is no Ultra frame: counted, nothing armed")
bad = bytearray(nop)
bad[36] ^= 0x40
chain_decode(bytes(bad), 320, 150000)
check(ucv("uc_nonultra") == 2, "CHAIN: a broken header is no Ultra frame")

# a POLL for answer index 1 of 2 at 500 kbaud: the answer's start word and the window's end from t0 and the clock
w(ux + O["ux_r_have"], 1, 1)                    # the clock's rate is measured (an unmeasured one answers no POLL: below)
wpu24 = e.rd(ux + O["ux_wpu24"], 4)
known = e.rd(ux + O["ux_r_have"], 1)
pitch = 330 * 2 + 130
end_w = 200000
chain_decode(poll_r(1, 2, H["ULTRA_PAGE_STATUS"], 1, 0x37), 320, end_w)
a = e.peek(ANS, 32)
tx_us = 1000 + pitch * 1 + (0 if known else 10)
check(e.rd(DL + O["dl_ch_arm"], 1) == 1 and e.rd(DL + O["dl_ch_tx"], 4) == end_w + (tx_us * wpu24 >> 24) and
      e.rd(DL + O["dl_ch_win"], 4) == end_w + ((1000 + pitch * 2 + 200) * wpu24 >> 24),
      "CHAIN: a POLL for index 1 of 2: armed, starts at t0 + %d us (word %d), window to t0 + %d us" % (tx_us, e.rd(DL + O["dl_ch_tx"], 4) - end_w,
                                                                                                    1000 + pitch * 2 + 200))
check(a[0] == H["ULTRA_RSP_SYNC"] and a[1] == H["ULTRA_PAGE_STATUS"] and a[2] >> 4 == 1 and a[3] == 0x37 and u(a, 4, 2) == 0xBEEF and
      u(a, 30, 2) == crc16(a[:30]), "CHAIN: the answer built: SYNC / page / index 1 / SEQ / UID16 / CRC")
check(e.rd(A("ud_brr"), 2) == brr[1] and ucv("uc_windows") == 1, "CHAIN: the divider for 500 kbaud, one window counted")
# RELAY_ONLY: the window is flagged, nothing armed
chain_decode(poll_r(H["ULTRA_POLL_RELAY_ONLY"], 1, H["ULTRA_PAGE_STATUS"], 0, 0x38), 320, 210000)
check(e.rd(DL + O["dl_ch_arm"], 1) == 0 and e.rd(DL + O["dl_ch_win"], 4) == 210000 + ((1000 + 460 * 1 + 200) * wpu24 >> 24) and ucv("uc_windows") == 2,
      "CHAIN: RELAY_ONLY: a window, no answer")
# a frame without RW: a CONFIG slot is acted on (CONFIG 140 = 1 forces STREAM), the window stays 0
chain_decode(frame(H["ULTRA_CONFIG"], bytes([140, 0, 1]), 12, 0, 10100), 320, 220000)
check(e.rd(ux + O["ux_cfg"] + 2 * 11, 2) == 1 and e.rd(DL + O["dl_ch_win"], 4) == 0, "CHAIN: CONFIG 140 taken from a chain frame")
# a 3-slot frame: slot 0 acted on, counted as not the last board
three = frame(H["ULTRA_NOP"], b"", 2, 0, 10200)[:32] + frame(H["ULTRA_NOP"], b"", 3, 0)[:32] + frame(H["ULTRA_NOP"], b"", 4, 0)[:32] + \
    frame(H["ULTRA_NOP"], b"", 4, 0, 10200)[32:]
chain_decode(three, 3 * 256 + 64, 230000)
check(ucv("uc_longs") == 1 and ucv("uc_slots", 1) == 3 and ucv("uc_flags", 1) & H["ULTRA_PXL_F_LONG"], "CHAIN: a 3-slot frame is counted as not the last board")
# an UPD frame: 384 bits, header only
d, _ = answer(H["ULTRA_PAGE_HW_FX"], bytes([H["ULTRA_PXG_PART_LINE"]]), 0)
check(d[H["ULTRA_PXL_MODE"]] == 2 and u(d, H["ULTRA_PXL_FRAMES"], 2) == ucv("uc_frames") and u(d, H["ULTRA_PXL_NONULTRA"], 2) == 2 and
      u(d, H["ULTRA_PXL_LONG"], 2) == 1 and u(d, H["ULTRA_PXL_WINDOWS"], 2) == 2 and d[H["ULTRA_PXL_FLAGS"]] & H["ULTRA_PXL_F_LONG"] and
      d[H["ULTRA_PXL_FLAGS"]] & H["ULTRA_PXL_F_NONULTRA"] and u(d, H["ULTRA_PXL_BITS"] , 3) == 832 and d[H["ULTRA_PXL_SLOTS"]] == 3,
      "CHAIN: page 0x81 part 3 shows it: mode CHAIN, frames %d, non-Ultra 2, long 1, windows 2, bits 832" % u(d, H["ULTRA_PXL_FRAMES"], 2))
# a POLL before the clock's rate is measured: the window is counted, the answer is not armed (the HSI would shift its start out of the window), a miss is counted
w(ux + O["ux_r_have"], 1, 0)
m0 = ucv("uc_missed")
chain_decode(poll_r(0, 1, H["ULTRA_PAGE_STATUS"], 0, 0x39), 320, 240000)
check(e.rd(DL + O["dl_ch_arm"], 1) == 0 and ucv("uc_missed") == m0 + 1 and ucv("uc_windows") == 3 and
      e.rd(DL + O["dl_ch_win"], 4) == 240000 + ((1000 + 460 * 1 + 200) * wpu24 >> 24),
      "CHAIN: a POLL before the rate is measured: window counted, no answer armed, counted as missed")
w(ux + O["ux_r_have"], 1, 1)


# ---------------------------------------------------------------------------------------------- the status LED
def led_levels(ms, step=10):
    """led_service for ms, in steps: [(t, level)] at each change of PB15 (GPIOB BSHR: bit 15 on, bit 31 off)"""
    out, t = [], 0
    for _ in range(ms // step):
        e.writes.clear()
        e.call("led_service", [step])
        t += step
        for reg, val in e.writes:
            if reg == 0x40010C10 and val & (1 << 15):
                out.append((t, 1))
            elif reg == 0x40010C10 and val & (1 << 31):
                out.append((t, 0))
    return out


w(A("led_lvl"), 1, 1)                           # (main sets it with the LED on)
f = frame(H["ULTRA_IDENTIFY"], bytes([H["ULTRA_IDENT_VERSION"]]), 0x40, 0)
r, _ = run(f)
check(r == 0 and e.rd(A("led_lead"), 2) == 1000, "IDENTIFY 5: the version's code starts (1 s off first)")
run(f)                                          # a copy: dropped (the same pattern runs on)
ch = led_levels(16000)
check(ch[:1] == [(10, 0)] and (1010, 1) in ch, "IDENTIFY 5: LED off for 1 s, then the pattern: %s" % ch[:3])
n_on = sum(1 for _, lv in ch if lv == 1)
check(n_on >= 3 + 1 + 1 + 2 and ch[-1][1] == 1 and e.rd(A("led_lead"), 2) == 0,
      "IDENTIFY 5: 0.7.1 shown (3 fast, 1, 2, 2 blinks), the LED on again: %d on-edges" % n_on)
# CHAIN: frames that are no Ultra frame (a pixel stream on a chain board) blink the status LED twice (a fault pattern, at most every 30 s)
w(A("led_lvl"), 1, 1)
e.poke(UC, bytes(e.sym("uc")[1]))
w(UC + O["uc_mode"], 1, 2)
w(UC + O["uc_nonultra"], 2, 5)
w(A("wrong_seen"), 2, 0)
w(A("wrong_s"), 4, 0)
w(A("up_s"), 4, 100)
w(A("led_lead"), 2, 0)
e.call("second_service", [], max_steps=2_000_000)
check(e.rd(A("led_lead"), 2) == 1000 and e.rd(A("wrong_seen"), 2) == 5, "CHAIN: non-Ultra frames start the LED's fault code (1 s off first)")
ch = led_levels(9000)
n_on = sum(1 for _, lv in ch if lv == 1)
check(ch[:1] == [(10, 0)] and n_on >= 4 and ch[-1][1] == 1, "CHAIN: the 2-blink fault code runs and ends with the LED on: %s" % ch[:8])
w(UC + O["uc_nonultra"], 2, 6)
w(A("up_s"), 4, 110)
e.call("second_service", [], max_steps=2_000_000)
check(e.rd(A("led_lead"), 2) == 0, "CHAIN: the code is not repeated within 30 s")
for act, name in ((3, "SAME"), (4, "NACK")):     # UPD_APP_SHOW_SAME / _REJECTED
    e.call("upd_action", [act])
    ch = led_levels(5000)
    check(ch[:2] == [(10, 0), (1010, 1)] and ch[-1][1] == 1 and len(ch) == 4,
          "an update's %s: 1 s off, one flash, on again: %s" % (name, ch))

print("a chain frame decoded in one go by ud_chain_decode (udc_classify: the rising-edge decode of the head and of the snapshot's tail, the CRCs, the slot's action, the answer built; the board spreads the head over the frame): %s cycles = %s us at 144 MHz"
      % (sorted(decode_cycles)[len(decode_cycles) // 2], sorted(decode_cycles)[len(decode_cycles) // 2] // 144))
print("ud_pages: %d checks, %d failed" % (checks, fails))
sys.exit(1 if fails else 0)
