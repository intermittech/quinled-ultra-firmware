/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* SCENE has two level sets; removed effect commands cannot modify either. */
static void test_scenefx(void)
{
    uint8_t slot[32],roles[5]={ULTRA_ROLE_BRIGHTNESS,0,0,0,0};
    uint16_t lo[5]={0},hi[5]={65535,65535,65535,65535,65535};
    scene_t stored,result;ultra_t before;scene_rec_t rec;
    ultra_init(&U,1);scene_clear(&stored);
    ultra_slot_light_def(slot,0,1,roles,1000,0,lo,hi,0);ultra_on_slot(&U,slot,0);
    ultra_slot_scene(slot,0,2,0,3,40000,hi,hi);ultra_on_slot(&U,slot,0);
    ultra_slot_scene(slot,0,3,1,7,10000,hi,hi);ultra_on_slot(&U,slot,0);
    ultra_scene(&U,&stored,&result);
    CHECK(result.v[0]==40000 && result.fade==3,"power-on levels and fade");
    CHECK(result.loss_v[0]==10000 && result.loss_fade==7,"independent signal-loss levels and fade");
    scene_log_pack(&result,17,&rec);CHECK(scene_rec_valid((const uint8_t *)&rec),"both levels persist atomically");
    rec.format=3;rec.crc=crc8_smbus((const uint8_t *)&rec,sizeof(rec)-2);CHECK(!scene_rec_valid((const uint8_t *)&rec),"old record rejected");
    before=U;slot[ULTRA_PAYLOAD]=2;ultra_slot_seal(slot);
    CHECK(!ultra_on_slot(&U,slot,0) && !memcmp(&U,&before,sizeof(U)),"reserved scene selector is atomic no-op");
    test_ultra_effects();
}
