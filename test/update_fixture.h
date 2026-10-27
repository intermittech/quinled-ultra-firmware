/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Wire-3 identity used by the robustness models. */
#ifndef TEST_UPDATE_FIXTURE_H
#define TEST_UPDATE_FIXTURE_H
static const upd_bl_info_t robust_boot = {{'Q','D','B','2'},1,
    {{1,1,1,1,3,0,0,UPD_REV_D2A_R04,8,0,0x21,0,0,0,0xDF,0}}, {0x88,10,20},5,{0}};
static void robust_desc(uint8_t *d, uint32_t version, uint8_t hw, uint16_t revmask, uint8_t min_bl)
{
    uint32_t i;
    memset(d,0,UPD_DESC_BYTES);memcpy(d,"QDU6",4);d[UPD_D_VERSION]=UPD_DESC_VERSION;
    memcpy(d+UPD_D_TARGET,robust_boot.target.bytes,7);d[UPD_D_TARGET]=hw;
    for(i=0;i<16u && !(revmask & (1u<<i));++i) {}
    d[UPD_D_REV_MIN]=d[UPD_D_REV_MAX]=(uint8_t)i;
    upd_put32(d+UPD_D_FW_VERSION,version);upd_put32(d+UPD_D_BODY_LEN,30548u);
    upd_put32(d+UPD_D_BODY_CRC,0x12345678u);upd_put32(d+UPD_D_LOAD,UPD_APP_BODY);
    upd_put32(d+UPD_D_ENTRY,UPD_APP_ENTRY);d[UPD_D_MIN_BL]=min_bl;
    upd_put32(d+UPD_D_CRC,upd_crc32_words(0xFFFFFFFFu,d,15));
}
#endif
