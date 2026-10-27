/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Included by the application soak: real AUTO, DMA decode and forwarding
 * decisions against the pinned sender traces at each tick phase. */
#include "protocol/golden_traces.h"

static uint64_t m1b_start(unsigned phase)
{
    return (sim_ns/50000u+1u)*50000u+(uint64_t)phase*1000u;
}

static void m1b_frame(const uint8_t *bytes, unsigned n, unsigned phase)
{
    uint64_t start=m1b_start(phase), end;
    wire_frame(bytes,n*8u,start);
    end=wf_fall(n*8u-1u)+500000u;
    while(sim_ns<end) run_ticks(1);
}

static void m1b_slots(const uint8_t *slots, unsigned n, unsigned phase)
{
    uint8_t f[104];
    unsigned i;
    uint64_t start=m1b_start(phase), last;
    memcpy(f,slots,n*32u);
    ultra_build_header(f+n*32u,0);
    for(i=0;i<4;i++) {
        unsigned high=(f[n*32u+7u]&1u)?wf_one:wf_zero;
        last=start+((uint64_t)(n*256u+63u)*wf_period+high)*1000000000u/wf_clock;
        ultra_build_header(f+n*32u,host_us(last));
    }
    m1b_frame(f,n*32u+8u,phase);
}

static void sc_m1bprofile(unsigned profile, unsigned phase)
{
    const ultra_trace_t *base=&ultra_golden_traces[profile*3u];
    uint8_t slots[64], f[72], data[512];
    unsigned i, k, goldens=0;
    uint32_t headers, dec0, drops;
    CHECK(profile<4u && phase<50u,"profile and phase in range");
    if(profile>=4u || phase>=50u) return;
    wf_clock=base->clock_hz; wf_period=base->period;
    wf_zero=base->zero; wf_one=base->one;
    /* Power up inside unrelated traffic. Only later complete frames may
     * acquire AUTO; no seeded receiver or clock state. */
    memset(data,0x55,sizeof data);
    wire_frame(data,sizeof data*8u,0);
    sim_ns=100000u;
    ring_words=sim_ns*9u/4000u;
    while(wf_fall(wf_next_fall)<=sim_ns) wf_next_fall++;
    boot(0,0,0);
    run_ms(25);
    CHECK(!ultra_on,"partial frame at boot cannot acquire Ultra");
    {
        uint8_t h[8];
        unsigned zero=wf_zero;
        ultra_build_header(h,0); h[7]^=1u;
        for(i=0;i<3u;i++) m1b_frame(h,8,phase);
        CHECK(!ultra_on && !line_lock.good,"bad header CRC cannot acquire");
        ultra_build_header(h,0); wf_clock*=4u; wf_period*=4u; wf_one*=4u; wf_zero=1;
        for(i=0;i<3u;i++) m1b_frame(h,8,phase);
        CHECK(!ultra_on && !line_lock.good,"out-of-bounds header pulses cannot acquire");
        wf_clock/=4u; wf_period/=4u; wf_one/=4u; wf_zero=zero;
        m1b_frame(h,8,phase);
        CHECK(!ultra_on && line_lock.good==1u,"one header cannot lock");
        m1b_frame(h,8,phase);
        CHECK(!ultra_on && line_lock.good==2u,"two headers cannot lock");
        m1b_frame(h,8,phase);
        CHECK(ultra_on && line_lock.profile==profile+4u && line_lock.good==3u,
              "three headers establish tentative keepalive (%u/%u/%u)",
              ultra_on,line_lock.profile,line_lock.good);
        CHECK(!ul.clk.synced && !ul.clk.rated && !line_stats.trained &&
              !ws_line_ready(&line_lock,ms_now) && !ultra_seen,
              "header-only keepalive cannot qualify timing or apply a slot");
        for(i=0;i<60u;i++) { m1b_frame(h,8,phase); run_ms(20); }
        CHECK(!ul.clk.synced && !ul.clk.rated && !line_stats.trained &&
              !ws_line_ready(&line_lock,ms_now) && !signal_lost && !out_now[0],
              "sustained header-only traffic keeps darkness without qualification");
        {
            unsigned timeout=ul.cfg[ULTRA_CFG_LOSS_TIMEOUT], loss=eff.signal_loss;
            ul.cfg[ULTRA_CFG_LOSS_TIMEOUT]=5u; eff.signal_loss=3u;
            scene_want.loss_mask=1u; scene_want.loss_v[0]=10000u;
            run_ms(2500); /* loss timeout, scene fade and output soft start */
            CHECK(signal_lost && out_now[0]==10000u,
                  "actual header-only signal loss reaches configured level (%u)",out_now[0]);
            ul.cfg[ULTRA_CFG_LOSS_TIMEOUT]=(uint16_t)timeout; eff.signal_loss=(uint8_t)loss;
        }
    }
    ultra_slot_dark(slots);
    for(i=0;i<10u;i++) m1b_slots(slots,1,phase);
    for(i=0;i<5u;i++) {
        static const uint8_t patterns[]={0x55,0xaa,0,0xff,0x55};
        memset(slots,0,32); memset(slots+3,patterns[i],27);
        ultra_slot_seal(slots);
        m1b_slots(slots,1,phase);
        if(i<4u) run_ms(250);
    }
    CHECK(ultra_on && line_lock.profile==profile && line_lock.good>=3u &&
          line_stats.trained,"AUTO trained at profile %u (%u/%u/%u/%u)",
          profile,ultra_on,line_lock.profile,line_lock.good,line_stats.trained);
    dec0=stat.decoded; drops=ovr[0]+ovr[1];
    {
        const uint8_t roles[5]={ULTRA_ROLE_BRIGHTNESS,0,0,0,0};
        const uint16_t z[5]={0}, a[5]={65535,65535,65535,65535,65535};
        ultra_slot_light_def(slots,0,70,roles,0,0,z,a,0);
        for(i=0;i<3u;i++) m1b_slots(slots,1,phase);
        ultra_slot_set(slots,0,71,0,16384,a,a,0,0);
        for(i=0;i<3u;i++) m1b_slots(slots,1,phase);
        run_ms(1000); /* the configured soft start must settle */
        CHECK(out_now[0]>=16383u && out_now[0]<=16385u,
              "profile %u command output %u",profile,out_now[0]);
    }
    /* Hardware forwarding toggles at the falling-edge counts from the
     * application's plan. Compare every downstream bit, including trailer. */
    ultra_slot_nop(slots);
    for(i=0;i<32u;i++) slots[32+i]=(uint8_t)(i*73u+19u);
    ultra_slot_seal(slots+32);
    memcpy(f,slots,64); ultra_build_header(f+64,host_us(m1b_start(phase)));
    memset(forward_bytes,0,sizeof forward_bytes); forward_bits=0; forward_watch=1;
    m1b_frame(f,sizeof f,phase);
    forward_watch=0;
    CHECK(forward_bits==320u && !memcmp(forward_bytes,f+32,40),
          "profile %u forwards the complete next slot and trailer (%u bits)",profile,forward_bits);
    for(k=0;k<sizeof ultra_golden_traces/sizeof ultra_golden_traces[0];k++) {
        const ultra_trace_t *v=&ultra_golden_traces[k];
        if(v->profile!=profile) continue;
        wf_clock=v->clock_hz; wf_period=v->period; wf_zero=v->zero; wf_one=v->one;
        headers=stat.ultra_headers;
        m1b_frame(v->frame,sizeof v->frame,phase);
        CHECK(stat.ultra_headers==headers+1u && !memcmp(frx.slot,v->frame,32) &&
              !memcmp(line_slot,v->frame,sizeof v->frame),
              "sender golden %u, profile %u phase %u (headers +%u, slot %d, prefix %d, own %u)",
              k,profile,phase,stat.ultra_headers-headers,memcmp(frx.slot,v->frame,32),
              memcmp(line_slot,v->frame,sizeof v->frame),cap_own);
        {
            uint32_t updates=upd.frames;
            for(i=0;i<5u;i++) m1b_frame(v->update,sizeof v->update,phase);
            CHECK(upd_app_transparent(&upd) && upd.frames>=updates+2u &&
                  !memcmp(line_slot,v->update,sizeof v->update),
                  "update golden %u, profile %u phase %u (%u payloads)",
                  k,profile,phase,upd.frames-updates);
            m1b_frame(v->frame,sizeof v->frame,phase);
            CHECK(!upd_app_transparent(&upd),"normal frame ends update transparency");
            /* The ISR has already rearmed the just-ended frame's plan;
             * one more normal refresh installs the published slot plan. */
            m1b_frame(v->frame,sizeof v->frame,phase);
        }
        goldens++;
    }
    CHECK(ovr[0]+ovr[1]==drops && stat.decoded>=dec0+7u+goldens,
          "complete frames retained, decoded %u, drops %u",stat.decoded-dec0,ovr[0]+ovr[1]-drops);
    /* Establish a fresh rate after the goldens' independent TIME origins. */
    host_on=1; host_next=sim_ticks; run_ms(1500); host_on=0;
#if BOARD_RETURN
    {
        rc_obs_t o;
        uint8_t poll[32];
        rc_phase_ns=phase*1000u+1u;
        rc_poll_build(poll,99,0,1,ULTRA_PAGE_STATUS,0,0,0);
        rc_window(poll,1,ULTRA_HDR_RW,0,0,&o);
        CHECK(rc_timing_ok(&o,poll) && rc_answer_ok(&o,poll)==ULTRA_PAGE_STATUS,
              "profile %u return window at phase %u",profile,phase);
        printf("m1b timing: turn %.3f check %.3f answer %.3f forward %.3f us\n",
               o.turn_us,o.check_us,o.tx_us,o.fwd_us);
    }
#endif
    CHECK(!events_seen,"profile traffic creates no power events");
    printf("m1b profile %u phase %u: %u sender goldens, AUTO, late join, forwarding\n",profile,phase,goldens);
}
