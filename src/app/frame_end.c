/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* frame_end.c: the main loop's side of a frame: the capture's decode (process_capture), the frame end's decisions and
 * the next gate plan (process_frame_end).
 * A unit of src/app/main.c: main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

static void process_capture(void)
{
    BARRIER();
    uint8_t *bits = (uint8_t *)core_scratch;
    uint32_t skip = cap_skip, want = cap_bits - skip;
    proto_t p = (proto_t)cap_proto;
    slot_t slot;
    int n;

    BARRIER();                             /* din_copy: read after cap_ready (the ISR's copy is complete) */
    if (cap_own == PROTO_UPD_BITS && skip == 0u) {
        upd_capture();                     /* a whole update frame (the transparent plan): not a slot */
        BARRIER();
        cap_ready = 0;
        return;
    }
    if (want - 1u >= PROTO_MAX_BITS) {       /* 0 or too many (also cap_bits < cap_skip); 0 = a WS2915 frame
                                              that ended at its header (no 0-bit decode, not counted) */
        BARRIER();
        cap_ready = 0;
        return;
    }
    n = capture_decode(want,skip,bits);
    if (n != (int)want) {
        BARRIER();
        cap_ready = 0;
        if (n < 0 && -n < 6) {
            stat.dec_err[-n]++;
        }
        return;
    }
    stat.decoded++;
    n = (int)frx_capture(&frx, cap_frame, p, bits, want, cap_own, skip, &slot);
    BARRIER();                             /* release only after every sample and metadata read */
    cap_ready = 0;
#if BOARD_RETURN
    if (frx.have_ultra && rc_poll_parse(frx.slot) == 0) {
        rc_pend = 1;                       /* a POLL: a window may follow this frame's end (rc_arm) */
        rc_frame = frx.cap_frame;
    }
#endif
    if ((uint32_t)n & FRX_SHOW_WS) {
        handle_slot(&slot, p);
    }
}

static void process_frame_end(void)
{
    BARRIER();
    uint8_t tail[8], i;
    const uint8_t *t8 = 0;
    frx_end_t e;
    uint32_t end_tick = fe_tick, total = fe_bits, keepalive = 0;
#if BOARD_RETURN
    int32_t rc_d = -1;
    uint32_t rc_odd = 0;
#endif
    BARRIER();                             /* tail_copy: read after fe_ready */
    if(cap_own>=PROTO_DETECT_BITS) {
        if(line_profile>=4u) {
            if(line_capture.bits==total && ws_line_decode(&line_capture,din_copy,64u,tail,1)==64) t8=tail;
        } else {
            uint32_t nw=(ws_line_threshold(line_profile)*2u*67u+15u)/16u+HINT_WORDS;
            if(ws_rise_tail(din_ring,DIN_RING_WORDS-1u,line_tail-nw,nw,ws_line_threshold(line_profile),tail)>=64) t8=tail;
        }
    } else if (tail_words != 0u &&
        PERF_CALL(PERF_HEADER, ws_decode_last(tail_copy, (uint32_t)tail_words * 16u, &ws_tm, tail)) >= (int)ULTRA_HDR_BITS) {
        t8 = tail;                         /* the frame's last 64 bits: an Ultra header? */
    }
#if BOARD_RETURN
    if(rc_pend && fe_frame==rc_frame && t8 && cap_own>=PROTO_DETECT_BITS) {
        if(line_profile>=4u)
            rc_d=(int32_t)(((uint32_t)rc_fe_pos*16u-(cap_start*16u+line_capture.fall))&(DIN_RING_WORDS*16u-1u))
                 +(int32_t)(TAIL_PAST-(rc_fe_pos&1u))*16;
        else {
            uint32_t nw=HINT_WORDS+16u, start=(line_tail-nw)&(DIN_RING_WORDS-1u);
            uint32_t i;
            for(i=0;i<nw;i++) tail_copy[i]=din_ring[(start+i)&(DIN_RING_WORDS-1u)];
            rc_d=rc_last_edge(tail_copy,nw);
        }
        rc_odd=rc_fe_pos&1u;
    }
    if (rc_pend && fe_frame == rc_frame && tail_words != 0u) {
        rc_d = rc_last_edge(tail_copy, tail_words);   /* t0: the frame's last edge, from the end of the tail copy */
        rc_odd = rc_fe_pos & 1u;
    }
#endif
    uint32_t slot_crc=frx.have_ultra?2u:0u;
    const uint8_t *label=frx.cap_frame==fe_frame && cap_own>=PROTO_DETECT_BITS && total>=ULTRA_SLOT_BITS?frx.slot:0;
    frx_frame_end(&frx, fe_frame, total, t8, &e);
    if(e.cls==PROTO_ULTRA && e.ultra_header && e.after==PROTO_NONE)
        e.plan=proto_gate_plan(PROTO_ULTRA);
    if(cap_own>=PROTO_DETECT_BITS) {
        uint32_t profile=line_profile<4u?line_profile:line_capture.profile;
        if(total==ULTRA_HDR_BITS) profile|=4u;
        uint32_t lock=ws_line_lock(&line_lock,profile,e.ultra_header,ms_now);
        if(lock&2u) { memset(&ul.clk,0,sizeof ul.clk); ul.clk.rate_q16=MCU_POLL_US<<16; }
        if(!(lock&1u)) e.ultra_sync=e.ultra_slot=0;
        keepalive=(lock&4u) && e.ultra_header;
        ls_frame(line_profile<4u?0:&line_capture,din_copy,label,e.ultra_header|slot_crc,ms_now,ovr[0]+ovr[1]);
    }
    BARRIER();
    fe_ready = 0;
#if BOARD_RETURN
    if (rc_pend) {
        rc_arm(&e, end_tick, rc_d, rc_odd);   /* first: its deadline is ULTRA_RW_DIR_MAX_US after the edge */
    }
#endif
    upd_app_frame_end(&upd, e.ultra_header, e.flags, ms_now);
    if (e.ultra_header && (e.flags & ULTRA_HDR_UPD)) {
        last_data_ms = ms_now;             /* an update session is signal too (lights hold / effects run) */
        signal_lost = 0;
    }
    if (e.after == PROTO_ULTRA && proto_tail_wanted(total)) {
        if (e.ultra_header) {
            stat.ultra_headers++;
        } else {
            stat.ultra_tail_err++;         /* bench: the tail window (TAIL_WORDS) or the hub's timing */
        }
    }
    if (e.show_ws) {
        handle_slot(&frx.ws, (proto_t)frx.ws_proto);   /* the stored protocol is confirmed by this frame */
    }
    if (e.close_gate) {
        gate_close_now();                  /* the stored protocol was wrong: forward nothing more of it */
    }
    if (e.after != e.before) {
        protocol_changed(e.before, e.after);
    }
    if(keepalive) {
        last_data_ms=ms_now; signal_lost=0; ul_used=1;
    }
    if (e.ultra_sync) {
        handle_ultra(e.host_us, end_tick, e.ultra_slot ? frx.slot : 0);   /* also on the frame that locked Ultra */
    }
    if (frx.ps.mode == PMODE_AUTO && frx.ps.locked != PROTO_NONE && !frx.ps.provisional &&
        frx.ps.locked != set.last_proto) {
        set.last_proto = (uint8_t)frx.ps.locked;   /* sticky auto: written only when the locked protocol changes */
        settings_changed();
    }
    i = (uint8_t)(plan_idx ^ 1u);          /* the ISR reads plan_buf[plan_idx] at the next frame end */
    plan_buf[i] = upd_app_transparent(&upd) ? proto_gate_plan_transparent() : e.plan;
    BARRIER();                             /* the plan and frx.ps are written before the ISR can take them */
    plan_idx = i;
}
