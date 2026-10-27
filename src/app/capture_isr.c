/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* capture_isr.c: the poll ISR and what it shares with the main loop: the frame tracking's events, the ring copies, the
 * gate plan, the return channel's window tick (BOARD_RETURN), the SRAM tick during a flash operation, the perf
 * counters.
 * A unit of src/app/main.c: main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

/* ------------------------------------------------------------------------------- shared with the poll ISR
 * The ISR writes volatile variables, ft (the main loop only with interrupts off) and the copies din_copy / tail_copy
 * (read after cap_ready / fe_ready, behind a BARRIER). frx and plan_buf are the main loop's; the ISR only reads them,
 * published by the BARRIER before plan_idx. */
static frx_t frx;                          /* protocol state and per-frame decisions (core/frame_rx.h) */
static gate_plan_t plan;                   /* plan of the frame in progress (the ISR's; the main loop writes it only
                                              with interrupts off) */
static gate_plan_t plan_buf[2];            /* next plan, written by the main loop (double buffer) */
static volatile uint8_t plan_idx;
static ftrack_t ft;                        /* frame tracking (ISR; the main loop only with interrupts off) */
static volatile uint8_t rx_ready;          /* main has set up the plans: the ISR starts tracking frames */
static volatile uint16_t cap_start;        /* ring word 60 us before the frame's first edge (inside the gap) */
static volatile uint16_t cap_words;
static volatile uint8_t cap_ready;
static uint8_t cap_accept;                 /* ISR: this frame owns the empty capture mailbox */
static volatile uint16_t cap_bits;         /* bits to decode */
static volatile uint16_t cap_own;          /* this board's bits under the plan of that frame */
static volatile uint16_t cap_skip;         /* WS2915: the capture starts after the header pause (hdr bits) */
static volatile uint8_t cap_proto;
static volatile uint32_t cap_frame;
static volatile uint8_t fe_ready;          /* frame end: total bits for the classifier */
static volatile uint32_t fe_bits;
static volatile uint32_t fe_frame;
static volatile uint32_t fe_tick;          /* tick of the frame's last edge (0-50 us after it): Ultra clock */
static volatile uint16_t tail_words;       /* tail_copy holds the frame's end (Ultra lengths only), 0 = none */
static uint32_t edge_tick;                 /* ISR: tick and ring position of the latest new edges */
static uint16_t edge_pos;
static volatile uint32_t ms_now;          /* 1 ms time base */
volatile uint32_t tick_now;               /* 50 us time base, also read by the SRAM flash deadline */
static uint32_t poll_sub;                  /* ticks into the ms (poll_isr, flash_poll_isr; 32 bits: no zero-extension
                                              every tick) */
static uint8_t adc_hold;                   /* the next tick's PA7 sample is held (the temperature sequence) */
static volatile uint32_t frames;
/* overruns (console `status` / `watch`, display only): [0] captures dropped (the last one not decoded yet),
 * [1] frame ends not handled (the last one still pending), both counted by the tick; [2] render ms late: the main loop's
 * ms came this many ms after the one before, not counted after a flash operation (ovr_on = 0: housekeeping) */
static volatile uint32_t ovr[3];
static uint16_t cpu_peak;                  /* (cpu_second below; the DIAG page's too) */
#if BOARD_RETURN
static uint16_t rc_peak_ma;                /* the input current's peak since the last ENERGY answer (bus side) */
#endif
static uint8_t ovr_on;
/* board current sense (core/isense.h): the tick sums the ADC samples and trips; the main loop limits (isense_apply
 * in outputs_update), calibrates and retries */
static isense_t is;
/* the reset breadcrumbs (core/cli.h bcrumb_t): RAM no reset clears (Link_sections.ld .noinit). Plain
 * stores, no read-modify-write: the tick writes pc, the main loop up_s, isr.S's fault handler pc / cause; read only at
 * boot, before the tick starts */
bcrumb_t bcrumb __attribute__((section(".noinit")));
typedef char bcrumb_offsets[(offsetof(bcrumb_t, pc) == 8u && offsetof(bcrumb_t, cause) == 12u) ? 1 : -1];   /* isr.S */

#define HINT_WORDS MCU_US2W(60u)                    /* 60 us at 36 MHz, 16 samples per word */
#define TAIL_WORDS MCU_US2W(256u)                    /* 256 us up to the frame's last edge: 64 bits at 1.5 us + hiccups + tick */
#define TAIL_PAST  4u                      /* words past the ring position of the last-edge tick */
/* the end of the frame, for the Ultra header (ws_decode_last); 4-byte aligned like din_copy: copy_run writes it in
 * 32-bit words (an array of uint16_t is only 2-byte aligned: today it lands on a word by the sizes before it) */
static uint16_t tail_copy[TAIL_WORDS] __attribute__((aligned(4)));
static ws_lock_t line_lock;
static ws_line_t line_capture;
static ws_rise_t line_prefix;
static uint8_t line_slot[48];
static uint32_t line_tag=0xffffffffu, line_consumed;
static volatile uint8_t line_active, line_profile;
static volatile uint16_t line_end, line_tail;

/* The main loop reads the DMA ring before its next wrap. The ISR publishes
 * frame boundaries only; it never scans pulse widths or queues runs. */
static void line_stream(void)
{
    uint32_t end, n, first, start;
    if(!line_active && !fe_ready) return;
    BARRIER();
    if(line_tag!=cap_frame) {
        line_tag=cap_frame; line_consumed=0;
        memset(&line_prefix,0,sizeof line_prefix);
        memset(line_slot,0,sizeof line_slot);
        ws_line_init(&line_capture);
    }
    start=cap_start;
    end=line_active?din_ring_pos():line_end;
    n=(end-start-line_consumed)&(DIN_RING_WORDS-1u);
    if(line_profile>=4u) {
        uint32_t at=(start+line_consumed)&(DIN_RING_WORDS-1u);
        first=DIN_RING_WORDS-at; if(first>n) first=n;
        ws_line_feed(&line_capture,din_copy,din_ring+at,first);
        ws_line_feed(&line_capture,din_copy,din_ring,n-first);
        line_consumed+=n;
    } else if(line_prefix.b<384u) {
        (void)ws_rise_ring(&line_prefix,din_ring,DIN_RING_WORDS-1u,start,line_consumed+n,
                           384u,ws_line_threshold(line_profile),line_slot);
        line_consumed=line_prefix.k;
    }
}
#if BOARD_RETURN
/* the return channel (core/rc.h): a POLL in this board's slot of an RW frame. The main loop arms the window at that
 * frame's end (rc_arm: the schedule from the frame's last edge, the answer built), the poll ISR runs it (rc_tick: turn
 * around, the burst, forward again at the window's end with the gate re-armed) and tracks no frames meanwhile: DIN
 * carries the relayed answers. */
enum { RC_IDLE = 0, RC_ARMED = 1, RC_TURNED = 2, RC_CHECKED = 3 };
static volatile uint8_t rc_state;          /* the main loop sets ARMED (from IDLE, after the rest), the ISR the rest */
static uint8_t rc_send;                    /* ISR: rc_ans still to send in this window */
static volatile uint8_t rc_contention;     /* a burst found the line taken since the last answer went out */
static volatile uint16_t rc_fe_pos;        /* ISR: the ring position at the last-edge tick of the frame end (fe_tick) */
static volatile uint16_t rc_answers, rc_aborts;   /* ISR: answers sent / aborted for contention (ERRORS page) */
static rc_sched_t rc_s;
static uint16_t rc_brr_v;
static uint8_t rc_ans[RC_ANSWER_BYTES];
#endif

/* k words s -> d, both 4-byte aligned: 32-bit copies, four per step (the poll ISR's longest path); returns the end */
static uint16_t *copy_run(uint16_t *d, const uint16_t *s, uint32_t k)
{
    uint32_t *dw = (uint32_t *)d;
    const uint32_t *sw = (const uint32_t *)s, *se = sw + (k >> 1);
    while (se - sw >= 4) {
        uint32_t a = sw[0], b = sw[1], c = sw[2], e = sw[3];
        dw[0] = a;
        dw[1] = b;
        dw[2] = c;
        dw[3] = e;
        dw += 4;
        sw += 4;
    }
    while (sw < se) {
        *dw++ = *sw++;
    }
    d = (uint16_t *)dw;
    if (k & 1u) {
        *d++ = *(const uint16_t *)sw;
    }
    return d;
}

/* ring words [start, end) -> dst (at most max words), two straight runs; returns the words copied (out of line: with
 * two callers LTO inlines it, and every path of poll_events saves one more register) */
static NOINLINE uint32_t ring_copy(uint16_t *dst, uint32_t start, uint32_t end, uint32_t max)
{
    uint32_t n = (end - start) & (DIN_RING_WORDS - 1u), first;
    if (n > max) {
        n = max;
    }
    first = DIN_RING_WORDS - start;
    if (first > n) {
        first = n;
    }
    dst = copy_run(dst, &din_ring[start], first);              /* start is even: aligned */
    (void)copy_run(dst, din_ring, n - first);                  /* when it wraps, first is even too (ring size even) */
    return n;
}

/* The UPD-transparent plan (open the whole frame) re-arms the gate closed: a frame end here is idle-only (frame_track:
 * FT_REARM_TICKS, 200 us or more without an edge, no length or CRC proof), so poll_isr opens it after GATE_QUIET_TICKS more
 * ticks without an edge, DIN LOW: >= 350 us of LOW in all (counted as if the end came 150 us after the last edge, the
 * figure the open kept before the re-arm moved), inside the inter-frame gap, never partway through a frame
 * (docs/timing.md "Update forwarding and entry"; decided 2026-10-23). A frame that starts first stays blocked,
 * whole. Every other plan re-arms with itself at once; a close (gate_close_now, the idle close, a flash operation) ends it.
 * Every re-arm and that open write FWD_EN only in a genuine quiet LOW (decided 2026-10-23): DIN read LOW,
 * then no edge counted since the tick's sample (a frame that began meanwhile is counted, even if DIN reads LOW between
 * its bits). Otherwise the frame end holds: FWD_EN keeps its level with its toggles stopped, so the frame in progress
 * passes or is held whole; the tracker takes it as a partial frame, and its own idle end re-arms. With valid traffic
 * (>= 280 us between frames, >= 500 us between Ultra frames) DIN is always LOW and quiet at the frame end: nothing changes
 * there. After an RW frame the hub's release (by 150 us) comes before the Ultra end: the gate holds it for the boards behind. */
#define GATE_QUIET_TICKS ((350u - (FT_REARM_TICKS - 1u) * MCU_POLL_US + MCU_POLL_US - 1u) / MCU_POLL_US)
static uint8_t gate_quiet;                 /* 0, or 1 + the quiet ticks since the transparent plan's re-arm */
static const gate_plan_t gate_shut = { 0, 0, 0, 0, 0, 0 };

/* re-arm the gate with the newest plan and make it the plan of the frame in progress (field by field: a struct copy is
 * a memcpy call): at a frame end and (the receiver) at the response window's end. One copy, out of line
 * where there are two callers */
#if BOARD_RETURN
static NOINLINE void plan_take(const gate_plan_t *g, uint16_t c)
#else
static inline void plan_take(const gate_plan_t *g, uint16_t c)
#endif
{
    if (!DIN_HIGH() && DIN_EDGES() == c) {
        gate_quiet = (uint8_t)(g->start_open && !g->t1);
        gate_rearm(gate_quiet ? &gate_shut : g);
    } else {
        gate_quiet = 0;
        gate_hold();
        ft.last_cnt = DIN_EDGES();         /* (the count runs on: what comes is no clean frame) */
        ft.armed = 0;
        ft.partial = 1;
    }
    plan.start_open = g->start_open;
    plan.t1 = g->t1;
    plan.t2 = g->t2;
    plan.own_bits = g->own_bits;
    plan.hdr_bits = g->hdr_bits;
    BARRIER();                             /* (no word copy of own_bits .. ws_end: offset 6 of a halfword-aligned plan) */
    plan.ws_end = g->ws_end;
}

/* the tracker's events of a tick but FT_EDGE (a few ticks per frame): out of line, so that the every-tick path of
 * poll_isr keeps no callee-saved registers (an idle-line test before the whole tracking
 * gave the idle tick the same and cost the busy ticks 11 cycles) */
static NOINLINE void poll_events(uint32_t ev, uint16_t c)
{
    /* one tick after our last bit (the DMA has surely written it), or at the end of a frame shorter than the capture:
     * decode what came (never both in one tick, nor with FT_CAP_START: core/frame_track.h; one copy) */
    if ((ev & (FT_CAP_COPY | FT_CAP_SHORT)) && cap_accept && cap_own<PROTO_DETECT_BITS) {
        cap_words = (uint16_t)ring_copy(din_copy, cap_start, din_ring_pos(), DIN_COPY_WORDS);
        cap_bits = ft.cap_bits;
        BARRIER();
        cap_ready = 1;
    }
    if ((ev & (FT_CAP_START | FT_CAP_DATA)) == FT_CAP_START) {
        cap_accept = (uint8_t)(!cap_ready && !fe_ready);
    }
    if ((ev & FT_CAP_START) && cap_accept) {               /* first edges (0-50 us ago): a ring position inside the reset gap */
        cap_start = (uint16_t)((din_ring_pos() - HINT_WORDS) & (DIN_RING_WORDS - 2u));   /* even */
        if (ev & FT_CAP_DATA) {
            cap_skip = plan.hdr_bits;      /* WS2915 data after the header pause (the gap is >= 100 us) */
        } else {
            cap_skip = 0;
            cap_own = plan.own_bits;
            cap_proto = (uint8_t)proto_active(&frx.ps);
            cap_frame = frames;
            line_profile=(uint8_t)(line_lock.good>=3u && line_stats.trained &&
                ms_now-line_lock.last_ms<ULTRA_RATE_LOCK_LOSS_MS ? line_lock.profile:255u);
            BARRIER();
            line_active=(uint8_t)(cap_own>=PROTO_DETECT_BITS);
        }
    } else if ((ev & (FT_CAP_START | FT_CAP_DATA)) == FT_CAP_START) {
        ovr[0]++;                          /* a frame's capture dropped: the last one not decoded yet */
    }
    if (ev & (FT_FRAME_END | FT_BOOT_ARM)) {
        const gate_plan_t *g = &plan_buf[plan_idx];   /* (the main loop writes only the other one) */
        if ((ev & (FT_FRAME_END | FT_PARTIAL)) == FT_FRAME_END && !fe_ready) {
            uint32_t total = c + ((uint32_t)ft.end_wraps << 16);
            fe_bits = total;                   /* the main loop classifies the frame and plans the next ones */
            fe_frame = frames;
            fe_tick = edge_tick;
#if BOARD_RETURN
            rc_fe_pos = edge_pos;
#endif
            /* the end of a frame of an Ultra length: its last 64 bits are the header (read in the main loop) */
            tail_words = cap_own<PROTO_DETECT_BITS && frx_tail_wanted(&frx, total)
                ? (uint16_t)ring_copy(tail_copy, (edge_pos + TAIL_PAST - TAIL_WORDS) & (DIN_RING_WORDS - 2u),
                                      edge_pos + TAIL_PAST, TAIL_WORDS)
                : 0u;
            line_tail=(uint16_t)(edge_pos+TAIL_PAST);
            line_end=(uint16_t)din_ring_pos();
            line_active=0;
            if(cap_accept && cap_own>=PROTO_DETECT_BITS) {
                cap_bits=(uint16_t)(total<cap_own?total:cap_own);
                BARRIER();
                cap_ready=1;
            }
            BARRIER();
            fe_ready = 1;
        } else if ((ev & (FT_FRAME_END | FT_PARTIAL)) == FT_FRAME_END) {
            ovr[1]++;                          /* the last frame end still pending: this one is not classified */
        }
        plan_take(g, c);                   /* the next frame starts >= 280 us after the last edge */
        if (ev & FT_FRAME_END) {
            frames++;
        }
    }
}

#if BOARD_RETURN
/* the response window, every tick from rc_arm to its end (core/rc.h): turn around, the turnaround check, this board's
 * burst, forward again. The gate is held closed for the whole window (the relayed answers are edges: a plan would open
 * it, forwarding UART traffic downstream as a frame); at the end it is re-armed with the newest plan, as at a frame
 * end. */
static NOINLINE void rc_tick(uint16_t c)
{
    uint32_t t = tick_now;
    BARRIER();                             /* acquire the schedule/answer published before RC_ARMED */
    if (rc_state == RC_ARMED) {
        if ((int32_t)(t - rc_s.dir_tick) >= 0) {
            gate_force_closed();
            rc_hw_dir(RC_DIR_ANSWER);
            rc_state = RC_TURNED;
        }
        return;
    }
    if (rc_state == RC_TURNED && (int32_t)(t - rc_s.check_tick) >= 0) {
        rc_state = RC_CHECKED;
        if (rc_hw_send(rc_ans, 0, rc_brr_v)) {
            rc_hw_dir(RC_DIR_QUIET);       /* the segment above is still driven: no relay, no answer this window */
            rc_send = 0;
            rc_contention = 1;
            rc_aborts++;
        }
    } else if (rc_send && (int32_t)(t - rc_s.tx_tick) >= 0) {
        rc_send = 0;
        if (rc_hw_send(rc_ans, RC_ANSWER_BYTES, rc_brr_v)) {
            rc_hw_dir(RC_DIR_QUIET);       /* the line was taken: no answer (the host retries) and, as at the check
, no more relay into the driver above for this window */
            rc_contention = 1;
            rc_aborts++;
        } else {
            rc_contention = 0;             /* (a CONTENTION before is in this answer's RFLAGS) */
            rc_answers++;
        }
    }
    if ((int32_t)(t - rc_s.end_tick) >= 0) {
        const gate_plan_t *g = &plan_buf[plan_idx];
        rc_hw_dir(RC_DIR_FORWARD);
        ftrack_window_end(&ft);            /* (first: a held re-arm marks the tracker after it) */
        plan_take(g, c);
        BARRIER();                         /* finish using the mailbox before main may refill it */
        rc_state = RC_IDLE;
    }
}
#endif

void poll_isr(void);                       /* 20 kHz tick, entered via isr.S (HPE + mret) */
void poll_isr(void)
{
    PERF_T(pt);                            /* (a PERF build: core/perf.h; first, before the acknowledge) */
    uint32_t c = DIN_EDGES(), own, ev;

    POLL_ACK();
    tick_now++;
    if (is.enabled) {                      /* (not before isense_setup at boot) */
        if (adc_hold) {
            /* the MCU temperature's injected sequence (84 us, started at the last tick) holds
             * PA7's conversion: this tick would read the value the last one counted, and one high sample would
             * trip as two. Not counted; the next tick has a new one (84 + 7 us < 100 us). */
            adc_hold = 0;
        } else if (isense_tick(&is, ISENSE_RAW())) {
            PWM_TRIP();                    /* a short: all outputs off now (pwm_force_off(1) inline), no table rebuild */
        }
    }
    if (++poll_sub >= POLL_HZ / 1000u) {
        poll_sub = 0;
        ms_now++;
        bcrumb.pc = TRAP_PC();             /* where the main loop is, once a ms */
        adc_hold = (uint8_t)hw_adc_ms();   /* the ADC's injected group: voltage channels / MCU temperature */
    }
#if BOARD_RETURN
    if (rc_state != RC_IDLE) {
        rc_tick((uint16_t)c);              /* the response window: no frame tracking until it ends */
        return;
    }
#endif
    if (gate_quiet) {                      /* the transparent plan's open (plan_take) */
        if (DIN_HIGH() || DIN_EDGES() != 0u) {   /* (DIN first, then the count: a quiet LOW all along) */
            gate_quiet = 0;                /* the next frame began first: it stays blocked */
        } else if (++gate_quiet > GATE_QUIET_TICKS) {
            gate_open();
            gate_quiet = 0;
        }
    }
    /* an idle line: ftrack_tick would return no event and change nothing (no edges, the count at 0
     * without wraps, no copy due, armed) */
    if (!rx_ready || (c == 0u && ft.last_cnt == 0u && ft.wraps == 0u && ft.cap_state != 2u && ft.armed)) {
        PERF_ADD(PERF_POLL, pt);
        return;
    }
    /* a provisional (stored) protocol captures an Ultra slot too: a different protocol locks at once (an update
     * session's transparent plan: still its whole 384-bit frame) */
    own = plan.own_bits;
    if (frx.ps.provisional && own < PROTO_DETECT_BITS) {
        own = PROTO_DETECT_BITS;
    }
    ev = ftrack_tick(&ft, (uint16_t)c, (uint16_t)own, plan.hdr_bits, plan.ws_end);
    if (ev & FT_EDGE) {                    /* the last edges so far: where the frame ends (in time and ring) */
        edge_tick = tick_now;
        edge_pos = (uint16_t)din_ring_pos();
    }
    if (ev & ~FT_EDGE) {
        poll_events(ev, (uint16_t)c);
    }
    PERF_ADD(PERF_POLL, pt);
}

#if PERF_BUILD
/* `perf` (core/perf.h, a bench build): the counts, the clock and the tick count at the last `perf` */
perf_t perf[PERF_N];
static uint32_t perf_clock0, perf_tick0;
#if PERF_BUILD == 2
/* the SysTick clock: ticks x cycles per tick + SysTick's count; a tick due but not counted yet (its flag up, the count
 * just past 0: the poll ISR before its acknowledge, or interrupts off) counts */
uint32_t perf_systick(void)
{
    uint32_t k = tick_now, c = POLL_CNT();
    if (MCU_POLL_PENDING() && c < MCU_HCLK_HZ / POLL_HZ / 2u) {
        k++;
    }
    return k * (MCU_HCLK_HZ / POLL_HZ) + c;
}
#endif

void perf_take(perf_t *dst, uint32_t *clock, uint32_t *ticks)
{
    uint32_t i, now;
    __disable_irq();
    now = PERF_NOW();
    *clock = now - perf_clock0;
    *ticks = tick_now - perf_tick0;
    perf_clock0 = now;
    perf_tick0 = tick_now;
    for (i = 0; i < PERF_N; i++) {
        dst[i] = perf[i];
        perf[i].n = perf[i].max = 0;
        perf[i].sum = 0;
    }
    __enable_irq();
}
#endif

/* The tick during a flash operation (hw.c flash_op: through VTF, from SRAM, the HCLK halved and SysTick's
 * compare with it): the time bases and the fast trip, as poll_isr (the same state: a high sample before the operation
 * still counts toward the two), nothing else (no frames: the gate is closed; no injected ADC sequences). */
RAMCODE void flash_poll_isr(void)
{
    POLL_ACK();
    gate_quiet = 0;                        /* (flash_op closed the gate: it stays closed until the next re-arm) */
    tick_now++;
    if (++poll_sub >= POLL_HZ / 1000u) {
        poll_sub = 0;
        ms_now++;
    }
    if (is.enabled) {
        if (adc_hold) {
            adc_hold = 0;
        } else if (isense_tick(&is, ISENSE_RAW())) {
            hw_flash_trip();               /* all outputs off (SRAM; pwm_forced as pwm_force_off(1)) */
        }
    }
}
