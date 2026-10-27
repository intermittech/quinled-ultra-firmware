/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The field offsets of main_ur_digital_m.c's state on the target, for test/test_ur_digital_m_pages.py: compiled with the BOARD=3 flags,
 * never linked; the test reads this table back from the object's .rodata.ud_offs section. */
#include "../src/app/main_ur_digital_m.c"
#include <stddef.h>

#define O(t, f) (unsigned)offsetof(t, f)
__attribute__((used, section(".rodata.ud_offs"))) const unsigned ud_offs[] = {
    0x55443300u,
    O(energy_t, lt), O(life_rec_t, on_s), O(life_rec_t, lit_s), O(life_rec_t, ups), O(life_rec_t, trips),
    O(life_rec_t, dips), O(life_rec_t, peak_t), O(life_rec_t, peak_a), O(life_rec_t, wd), O(life_rec_t, updates),
    O(life_rec_t, writes),
    O(upd_app_t, info), O(upd_app_info_t, hw_id), O(upd_app_info_t, hw_rev), O(upd_app_info_t, bl_version),
    O(vsense_t, valid), O(vsense_t, mv), O(mcutemp_t, c10), O(mcutemp_t, vdda_mv),
    O(dline_t, st), O(dline_stats_t, frames), O(dline_stats_t, bits_last), O(dline_stats_t, bits_min),
    O(dline_stats_t, bits_max), O(dline_stats_t, long_high), O(dline_stats_t, answered), O(dline_stats_t, marks),
    O(dline_stats_t, cap_drops), O(dline_t, wpos), O(dline_t, edge_w), O(dline_t, direct),
    O(ur_digital_m_stats_t, fps10), O(ur_digital_m_stats_t, faults), O(ur_digital_m_stats_t, polls), O(ur_digital_m_stats_t, headers),
    O(ur_digital_m_stats_t, tail_err), O(ur_digital_m_stats_t, decoded), O(ur_digital_m_stats_t, dec_err),
    O(dline_t, cap), O(upd_app_t, frames),
    O(udfx_t, cfg), O(udfx_t, npx), O(udfx_t, chip), O(ws_chip_t, bpp), O(udfx_t, seg), O(ux_seg_t, run), O(ux_seg_t, p),
    O(ultra_pixel_fx_t, effect), O(ultra_pixel_fx_t, brightness), O(udfx_t, want_local), O(udfx_t, want_stream),
    O(udfx_t, stored_have), O(udfx_t, save_due), O(udfx_t, seq5), O(udfx_t, lock), O(udfx_t, a_have), O(udfx_t, a_host),
    O(udfx_t, local), O(udfx_t, dark), O(udfx_t, save_ms),
    O(dline_t, ch_on), O(dline_t, ch_st), O(dline_t, ch_end), O(dline_t, ch_tx), O(dline_t, ch_win), O(dline_t, ch_bits),
    O(dline_t, ch_hn), O(dline_t, ch_tn), O(dline_t, ch_tw), O(dline_t, ch_arm), O(dline_t, ch_ready), O(dline_t, ch_enter), O(dline_t, ch_hold),
    O(dline_t, ch_req), O(udc_t, mode), O(udc_t, cfg), O(udc_t, flags), O(udc_t, slots), O(udc_t, frames), O(udc_t, nonultra),
    O(udc_t, longs), O(udc_t, windows), O(udc_t, answers_missed), O(udc_t, markers), O(udc_t, bits), O(udfx_t, chain),
    O(udfx_t, wpu24), O(udfx_t, r_have),
    sizeof(ux_seg_t),
    sizeof(life_rec_t), sizeof(upd_app_info_t),
    DLINE_CH_TAIL, O(dline_t, cap_n), O(dline_t, ch_cjs), O(dline_t, ch_profile),
    O(dline_t, line_profile), O(link_stats_t, trained), O(link_stats_t, profile),
    O(vdip_t, znum), O(vdip_t, zden), O(vdip_t, zsteps), O(vdip_t, count), O(vdip_t, in_dip), O(vdip_t, n_log),
    O(vdip_t, log), sizeof(((vdip_t *)0)->log),
    0x55443399u,
};
