# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The Ultra-Digital firmware's state offsets for the target tests (BOARD=3): test/ud_offsets.c compiled with the target compiler
($RISCV_GCC, else the xPack GCC in %LOCALAPPDATA%\\xPacks) and read back from the object's .rodata.ud_offs section.
test_ud_pages.py has the same two functions inline; the newer tests import them from here."""
import glob
import os
import struct
import subprocess
import tempfile
from pathlib import Path

FW = Path(__file__).resolve().parents[1]

KEYS = ("en_lt on_s lit_s ups trips dips peak_t peak_a wd updates writes upd_info info_hw_id info_hw_rev "
        "info_bl vs_valid vs_mv mt_c10 mt_vdda dl_st st_frames st_bits_last st_bits_min st_bits_max "
        "st_long_high st_answered st_marks st_cap_drops dl_wpos dl_edge_w dl_direct uds_fps10 uds_faults "
        "uds_polls uds_headers uds_tail_err uds_decoded uds_dec_err dl_cap upd_frames "
        "ux_cfg ux_npx ux_chip chip_bpp ux_seg seg_run seg_p fx_effect fx_bri ux_want_local ux_want_stream "
        "ux_stored_have ux_save_due ux_seq5 ux_lock ux_a_have ux_a_host ux_local ux_dark ux_save_ms "
        "dl_ch_on dl_ch_st dl_ch_end dl_ch_tx dl_ch_win dl_ch_bits dl_ch_hn dl_ch_tn dl_ch_tw dl_ch_arm dl_ch_ready "
        "dl_ch_enter dl_ch_hold dl_ch_req uc_mode uc_cfg uc_flags uc_slots uc_frames uc_nonultra uc_longs "
        "uc_windows uc_missed uc_markers uc_bits ux_chain ux_wpu24 ux_r_have "
        "sz_seg sz_life sz_info").split()


def target_gcc():
    if os.environ.get("RISCV_GCC"):
        return os.environ["RISCV_GCC"]
    xp = sorted(glob.glob(os.path.join(os.environ.get("LOCALAPPDATA", ""), "xPacks", "xpack-riscv-none-elf-gcc-*",
                                       "bin", "riscv-none-elf-gcc*")))
    if not xp:
        raise SystemExit("no target compiler: set RISCV_GCC")
    return xp[-1]


def offsets(elf):
    """the field offsets of main_ud.c's state, by the names in KEYS"""
    gen = Path(elf).parent / "gen"
    with tempfile.TemporaryDirectory() as td:
        obj = os.path.join(td, "ud_offsets.o")
        cmd = [target_gcc(), "-march=rv32imc_zicsr_zifencei", "-mabi=ilp32", "-Os", "-c", "-DCH32V20x_D6",
               "-DUPD_CRC_ULTRA", "-DBOARD=3", "-DMON_EXTRA=0", "-DCONSOLE_FULL=0", "-DUPPER_RECORDS=1",
               "-I" + str(FW / "vendor/wch"), "-I" + str(FW / "src/hal"), "-I" + str(FW / "src"), "-I" + str(gen),
               "-I" + str(FW / "src/core"), str(FW / "test/ud_offsets.c"), "-o", obj]
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
            assert len(KEYS) == len(v) - 2, "ud_target.KEYS and test/ud_offsets.c differ"
            return dict(zip(KEYS, v[1:-1]))
    raise SystemExit("ud_offsets.o: no .rodata.ud_offs")
