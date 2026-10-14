/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Independent round-4 tests. Included in the existing hardware model, not derived
 * from another reviewer's regressions. Each scenario runs in a fresh process. */
static void v_unit(void)
{
    uint32_t j, before;
    uint16_t samples[8] = {0, 0, 0xffff, 0xff00, 0, 0, 0, 0};
    uint8_t guard[8] = {0}, slot[32], tail[8];
    ws_timing_t timing = WS_TIMING_DEFAULT;
    frx_t r; frx_end_t end; slot_t ws;
    upd_app_t u; upd_app_info_t info = {0};
    ack_t a;
    int result;
    result = ws_decode_frame(samples,128,0,&timing,guard);
    CHECK(result == 0 && guard[0] == 0, "zero-bit decoder: result %d sentinel %u",result,guard[0]);
    /* (the energy wrap: superseded by the 0.6.3 lifetime record, test_energy.c covers it) */
    ultra_slot_nop(slot); ultra_build_header(tail,1000000);
    for(j=318;j<=386;j++) {
        frx_init(&r,PMODE_ULTRA,PROTO_NONE);
        (void)frx_capture(&r, 9, PROTO_ULTRA,slot,256,256,0,&ws);
        frx_frame_end(&r,9,j,tail,&end);
        CHECK(end.ultra_sync == (j==320), "enclosing length %u sync %u",j,end.ultra_sync);
    }
    upd_app_init(&u,&info);u.decided=1;u.have_half=1;u.result=UPD_APP_SHOW_SAME;
    upd_app_frame_end(&u,1,0,50);
    CHECK(!u.decided && !u.have_half && !u.result,"normal header expires decision and partial descriptor");
    u.decided=1;u.have_half=2;u.transparent=1;u.last_upd_ms=0xfffffff0u;
    (void)upd_app_poll(&u,(uint32_t)(0xfffffff0u+UPD_APP_IDLE_MS));
    CHECK(!u.decided && !u.have_half && !u.transparent,"timeout across ms wrap expires session");
    memset(&a,0,sizeof a);ack_start_version(&a,0x1c1c1c81u);
    CHECK(a.nseg==66,"version 28.28.28-rc.1 requires 66 segments; got %u",a.nseg);
    before=0;while(a.active && before<60000){uint8_t on;ack_tick(&a,&on);before++;}
    printf("version duration=%u ms segments=%u sizeof_ack=%u\n",before,a.nseg,(unsigned)sizeof a);
}
static void v_ultra(void)
{
    ultra_t u, other;
    uint8_t s[32], identify[32], def[32], fx[32], roles[5]={1,2,3,0,0};
    uint16_t lo[5]={0}, hi[5]={65535,65535,65535,65535,65535};
    uint16_t red[5]={65535,0,0,0,0}, blue[5]={0,0,65535,0,0}, b[5]={65535,65535,65535,65535,65535};
    uint8_t params[22]={0}; uint16_t out[5]; uint32_t j;
    ultra_init(&u,1);ultra_slot_light_def(def,0,1,roles,1000,0,lo,hi);
    ultra_on_slot(&u,def,0);
    ultra_slot_set(s,0,7,0,65535,red,b,0,0);ultra_on_slot(&u,s,0);ultra_render(&u,0,out);
    ultra_slot_calibrate(identify,9,25);ultra_on_slot(&u,identify,0);
    ultra_all_off(&u,20,0);
    CHECK(ultra_on_slot(&u,s,20)&ULTRA_EV_STATE,"same SET refresh must restore after signal loss");
    CHECK(!(ultra_on_slot(&u,identify,20)&ULTRA_EV_IDENTIFY),"signal loss must not replay a prior calibration");
    CHECK(!ultra_on_slot(&u,def,20),"unchanged LIGHT_DEF remains deduplicated");
    ultra_slot_effect(fx,0,11,ULTRA_FX_BREATHE,0,params,7);ultra_on_slot(&u,fx,20);
    ultra_all_off(&u,40,500);
    CHECK(ultra_on_slot(&u,fx,40)&ULTRA_EV_STATE,"same EFFECT refresh must restore after fade-loss");
    ultra_init(&u,1);ultra_on_slot(&u,def,0);
    ultra_slot_set(s,0,1,0,65535,red,b,0,0);ultra_on_slot(&u,s,0);
    ultra_slot_set(s,0,2,0,0,blue,b,1000,0);ultra_on_slot(&u,s,0);
    ultra_render(&u,10000,out);
    CHECK(out[0]>32000 && out[2]==0,"red fade-to-off stays red (R %u B %u)",out[0],out[2]);
    for(j=0;j<ULTRA_FX_COUNT;j++) {
        uint32_t n=ultra_fx_param_bytes((uint8_t)j),k;
        ultra_init(&u,1);ultra_init(&other,1);
        ultra_slot_effect(s,0,1,(uint8_t)j,0,params,22);ultra_on_slot(&u,s,0);
        for(k=n;k<22;k++)s[ULTRA_PAYLOAD+5+k]=0xb7;
        ultra_slot_seal(s);ultra_on_slot(&other,s,0);
        CHECK(!memcmp(u.light[0].pend_p,other.light[0].pend_p,22),"EFFECT %u spare bytes canonical",j);
        ultra_slot_scene_effect(s,0,2,(uint8_t)j,params,14);ultra_on_slot(&u,s,0);
        for(k=n;k<14;k++)s[ULTRA_PAYLOAD+1+k]=0xb7;
        ultra_slot_seal(s);ultra_on_slot(&other,s,0);
        CHECK(!memcmp(u.light[0].scene_fxp,other.light[0].scene_fxp,14),"SCENE_EFFECT %u spare bytes canonical",j);
    }
}
static void v_rx_setup(proto_mode_t mode,proto_t proto)
{
    frx_init(&frx,mode,PROTO_NONE);plan=plan_buf[0]=plan_buf[1]=proto_gate_plan(proto);
    memset(&ft,0,sizeof ft);ft.armed=1;rx_ready=1;frames=31;
    cap_ready=fe_ready=0;plan_idx=0;soak_tim4.CNT=0;
}
static void v_rx(void)
{
    uint32_t j;
    v_rx_setup(PMODE_WS2915,PROTO_WS2915_32);
    /* Short frame followed by a paused header before main consumes the short capture. */
    soak_tim4.CNT=1;poll_isr();for(j=0;j<3;j++)poll_isr();
    CHECK(cap_ready && cap_bits==1,"short capture fixture");
    soak_tim4.CNT=32;poll_isr();poll_isr();soak_tim4.CNT=112;poll_isr();
    printf("late short capture: bits=%u skip=%u frame=%u\n",cap_bits,cap_skip,cap_frame);
    process_capture();
    CHECK(!v_oversize,"no oversized decode (%u calls)",v_oversize);
    /* An occupied complete mailbox survives a full different frame. */
    v_rx_setup(PMODE_WS2805,PROTO_WS2805);
    cap_ready=fe_ready=1;cap_frame=19;cap_words=55;cap_bits=40;cap_skip=0;
    fe_frame=18;fe_bits=80;fe_tick=101;tail_words=16;din_copy[0]=0xa123;tail_copy[0]=0xb123;
    soak_tim4.CNT=120;poll_isr();for(j=0;j<4;j++)poll_isr();
    CHECK(cap_frame==19 && cap_words==55 && din_copy[0]==0xa123,"capture mailbox survives ISR producer");
    CHECK(fe_frame==18 && fe_bits==80 && fe_tick==101 && tail_copy[0]==0xb123,"end mailbox survives ISR producer");
    /* Synthetic inconsistent tags must be rejected even with ownership in place. */
    cap_bits=1;cap_skip=32;cap_own=112;cap_ready=1;process_capture();
    CHECK(!v_oversize,"defensive capacity bound rejects stale/malformed tags");
    CHECK(!cap_ready,"invalid capture releases mailbox");
}
static void v_long(void)
{
    uint32_t lengths[]={68000,72032,131072,65568},i,n,k;
    for(i=0;i<4;i++) {
        v_rx_setup(PMODE_AUTO,PROTO_NONE);n=lengths[i];
        for(k=40;k<n;k+=40){soak_tim4.CNT=(uint16_t)k;poll_isr();}
        soak_tim4.CNT=(uint16_t)n;poll_isr();for(k=0;k<3;k++)poll_isr();
        CHECK(fe_ready && fe_bits==n,"length %u published %u ready %u",n,fe_bits,fe_ready);
    }
}
static void v_reset(void)
{
    boot(0,0,0);run_ms(10);is.tripped=ISENSE_TRIP_FAST;sim_forced=1;
    console("defaults");
    CHECK(is.tripped!=0 && sim_forced,"defaults preserves trip/retry software state");
    run_ms(2500);CHECK(!sim_forced,"normal retry releases preserved trip");
}
static void v_gate(void)
{
    uint32_t j;
    for(j=0;j<2;j++) {
        v_rx_setup(PMODE_ULTRA,PROTO_ULTRA);
        plan=plan_buf[0]=plan_buf[1]=proto_gate_plan_transparent();
        upd.transparent=1;upd.last_upd_ms=100;ms_now=100+UPD_APP_IDLE_MS;
        soak_tim4.CNT=(uint16_t)(j?17:0);ft.cap_state=(uint8_t)(j?1:0);
        upd_service();
        CHECK(plan_buf[plan_idx].own_bits==ULTRA_SLOT_BITS,"timeout publishes normal plan (busy=%u)",j);
        CHECK(plan.own_bits==(j?PROTO_UPD_BITS:ULTRA_SLOT_BITS),"timeout active-frame deferral (busy=%u own=%u)",j,plan.own_bits);
    }
}
static void v_console(void)
{
    const char *p;uint32_t j;
    boot(0,0,0);run_ms(600);console("status");
    vs_sup.valid=1;vs_sup.mv=24000;vs_sup.min_mv=18000;vs_sup.max_mv=26000;
    console("version"); /* another command must not consume status's interval */
    con_out_n=con_in_r=con_in_w=0;
    for(p="status\r\n";*p;p++){con_in[con_in_w++]=*p;run_ms(10);}
    CHECK(strstr(con_out,"min 18.00 V, max 26.00 V")!=0,"typed status after version retains extrema: %s",strstr(con_out,"supply=")?strstr(con_out,"supply="):"missing");
    for(j=0;j<5;j++) run_ms(100);
    console("status");CHECK(stat.vs_supply_min_mv>23000,"next status restarts extrema");
}
static void v_clockback(void)
{
    ultra_t u; uint8_t s[32], roles[5]={1,2,3,0,0}, par[7]={0xb8,0x0b,0,0,0,0xff,0xff};
    uint16_t lo[5]={0},hi[5]={65535,65535,65535,65535,65535},out[5];uint32_t before;
    ultra_init(&u,1);ultra_on_header(&u,20000,1000000);
    ultra_slot_light_def(s,0,1,roles,1000,0,lo,hi);ultra_on_slot(&u,s,20000);
    ultra_slot_set(s,0,1,0,65535,hi,hi,0,0);ultra_on_slot(&u,s,20000);
    ultra_slot_effect(s,0,1,ULTRA_FX_BREATHE,0,par,7);ultra_on_slot(&u,s,20000);ultra_render(&u,20000,out);
    before=u.light[0].fx_t0;
    ultra_on_header(&u,20001,900050);ultra_render(&u,20001,out);
    printf("backwards 100 ms header: effect t0 %u -> %u, rebase flag %u, host_ms %u\n",before,u.light[0].fx_t0,u.light[0].fx_k,(unsigned)(ultra_host_us(&u,20001)/1000));
    CHECK(u.light[0].fx_t0==before,"a newly started effect must not undergo long-run rebase after a small backwards phase correction");
}
static int verify_scenario(const char *s)
{
    if(!strcmp(s,"vunit"))v_unit();
    else if(!strcmp(s,"vultra"))v_ultra();
    else if(!strcmp(s,"vrx"))v_rx();
    else if(!strcmp(s,"vlong"))v_long();
    else if(!strcmp(s,"vreset"))v_reset();
    else if(!strcmp(s,"vgate"))v_gate();
    else if(!strcmp(s,"vconsole"))v_console();
    else if(!strcmp(s,"vclockback"))v_clockback();
    else return 0;
    return 1;
}
