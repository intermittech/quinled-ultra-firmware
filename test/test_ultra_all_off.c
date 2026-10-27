/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Ordinary SET fades keep 16-bit resolution and can be interrupted. */
static void test_ultra_all_off(void)
{
    static const uint8_t mono[5]={ULTRA_ROLE_BRIGHTNESS,0,0,0,0};
    static const double a[5]={1,1,1,1,1};
    uint16_t before[5],out[5]; uint32_t ms;
    ultra_init(&U,1); def_light(&U,100,0,mono,1,0,0,1); set_light(&U,100,0,0,0.8,a,0,0);
    rend(&U,100,before); ultra_all_off(&U,tk(100),1000); rend(&U,100,out);
    CHECK(!memcmp(before,out,sizeof(out)),"fade starts continuously");
    rend(&U,600,out); CHECK(abs((int)out[0]-26214)<=2,"off midpoint");
    for(ms=601;ms<=1100;++ms) { uint16_t prev=out[0];rend(&U,ms,out);CHECK(out[0]<=prev,"off monotonic"); }
    CHECK(out[0]==0,"off ends exactly");
    set_light(&U,1200,0,0,1,a,0,0); ultra_all_off(&U,tk(1200),0);rend(&U,1200,out);CHECK(!out[0],"immediate off");
}
