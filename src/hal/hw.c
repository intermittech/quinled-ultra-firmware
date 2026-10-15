/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V203F8U6 drivers: PWM timers + dither DMA, the phase slew, DIN edge counter / gate, DIN sampler, ADC, return
 * channel, UART, the SRAM trip. The boot (clock, pins, peripherals) and the flash operation: hw_cold.c (round 9: built
 * with -msave-restore like the other cold files; what the two share: hw_priv.h). Register usage follows the WCH
 * CH32FV2x_V3xRM reference manual V2.5. Items marked BENCH need confirmation on real hardware. */
#include "ch32v20x.h"
#include "hw.h"
#include "hw_priv.h"
#include "../core/flashrec.h"
#include "../core/rc.h"
#include "../core/stagger.h"
#include "../core/scratch.h"
#include "../core/lto.h"
#include "../core/perf.h"

uint32_t SystemCoreClock = 144000000u;

/* DMA tables. Plain: TIM1 {CCR1, CCR2(, CCR3)}, TIM2 {CCR4}, TIM3 {CCR3, CCR4} per entry. Spread spectrum: every
 * entry also sets the period, burst from ATRLR: TIM1 {ARR, RPT, CCR1, CCR2}, TIM2 {ARR, -, CCR1, CCR2, CCR3 = 2,
 * CCR4}, TIM3 {ARR, -, CCR1 (capture: read-only), CCR2, CCR3, CCR4}. */
uint16_t pwm_tab1[PATTERN_MAX_LEN * 4];
uint16_t pwm_tab2[PATTERN_MAX_LEN * 6];
uint16_t pwm_tab3[PATTERN_MAX_LEN * 6];
uint16_t din_ring[DIN_RING_WORDS] __attribute__((aligned(16)));   /* (16: memset / copies take their fast path, */
uint16_t din_copy[DIN_COPY_WORDS] __attribute__((aligned(16)));   /* whatever the RAM layout; 0.6.3) */

uint32_t cur_steps;                           /* (cur_steps / cur_spread: hw_priv.h, hw_cold.c flash_op reads them) */
static uint8_t cur_stagger, cur_len;
uint8_t cur_spread;
static uint16_t spread_arr[PATTERN_MAX_LEN];  /* spread spectrum: period - 1 of each table entry */
static void pwm_outputs_mode(void);
volatile uint8_t pwm_forced;                  /* isense trip: all PWM outputs forced low */

/* ---------------------------------------------------------------------------------------------------- PWM */
/* Adaptive stagger: TIM2 / TIM3 move to new phase offsets (relative to TIM1) gradually. While a timer is off
 * target, its update interrupt changes the length of the next period by at most period / 512 (ARR, preloaded).
 * Only the off-time of left-aligned channels changes (< 0.2 % for that period). TIM3's right-aligned W2 ends at the
 * period end, so its CCR3 (just loaded by the update DMA burst for the same period) moves along: W2's pulse keeps
 * its width. */
static volatile uint16_t slew_tgt2, slew_tgt3;
volatile int32_t slew_pend2, slew_pend3;                /* (hw_priv.h: flash_op clears them) */

static uint32_t slew_step_max(void)
{
    uint32_t m = cur_steps / 512u;
    return m < 2u ? 2u : m;
}

void pwm_set_phase(uint16_t o2, uint16_t o3)
{
    if (!cur_stagger) {
        return;
    }
    slew_tgt2 = (uint16_t)(((uint32_t)o2 * cur_steps) >> 16);
    slew_tgt3 = (uint16_t)(((uint32_t)o3 * cur_steps) >> 16);
    BARRIER();                                           /* cur_* (pwm_start) are written before the ISRs run */
    TIM2->INTFR = (uint16_t)~TIM_UIF;
    TIM3->INTFR = (uint16_t)~TIM_UIF;
    TIM2->DMAINTENR |= TIM_UIE;
    TIM3->DMAINTENR |= TIM_UIE;
}

/* offset of a timer: TIM1's count at the timer's wrap (read just after the wrap, a few counts of skew) */
static uint32_t phase_of(uint32_t cnt_x)
{
    /* (c1 - cnt_x) mod cur_steps as two conditional additions instead of two divisions (fable-algo: 34 of the
     * interrupt's ~150 cycles): TIM1 counts below its period (ATRLR = cur_steps - 1; no spread here) and the timer's
     * count just after its own wrap is below cur_steps + its stretched period's step (< 2 cur_steps even when this
     * interrupt ran late) */
    int32_t d = (int32_t)TIM1->CNT - (int32_t)cnt_x;
    if (d < 0) d += (int32_t)cur_steps;
    if (d < 0) d += (int32_t)cur_steps;
    return (uint32_t)d;
}

/* spread spectrum: TIM1 and the timer run the same period sequence, offsets stay below 85 % of the period, so the
 * timer wrapped inside TIM1's current period: the offset is the plain difference */
static uint32_t phase_straight(uint32_t cnt_x)
{
    int32_t d = (int32_t)TIM1->CNT - (int32_t)cnt_x;
    return (uint32_t)(d < 0 ? d + (int32_t)cur_steps : d);
}

/* The table entry the DMA burst of this update wrote for the next period (`words` transfers, requested at the update
 * / at TIM2's count 2). 0.6.4 (gpt-hw H6): taken once that burst is complete. 0.6.3 read the channel's count when the
 * interrupt started: a burst still in progress (the usual case: the interrupt comes with the update) gave the entry
 * before, so the next period got the previous entry's length (and W2 its CCR3), a lasting phase error. The count sits
 * on an entry boundary before and after a burst: on a boundary at the first read, wait for it to move (an interrupt
 * entered after the timer's count SLEW_DMA_LATE found the burst done), then for the next boundary. Both waits end
 * at the timer's count, never unbounded. BENCH: the burst done well before SLEW_DMA_LATE with every DMA channel busy. */
#define SLEW_DMA_LATE 192u                               /* timer counts: 1.3 us at 144 MHz */
static inline __attribute__((always_inline)) uint32_t slew_entry(const volatile uint16_t *cnt,
                                                                 const volatile uint32_t *cntr, uint32_t words)
{
    uint32_t n = *cntr;
    if (n % words == 0u) {
        while (*cntr == n && *cnt < SLEW_DMA_LATE) {
        }
        n = *cntr;
    }
    while (n % words != 0u && *cnt < 2u * SLEW_DMA_LATE) {
        n = *cntr;
    }
    n = (words * cur_len - n) / words + cur_len - 1u;   /* the counter: words x cur_len .. 1, so cur_len - 1 .. */
    return n >= cur_len ? n - cur_len : n;               /* .. 2 cur_len - 1: mod cur_len (fable-algo: no division) */
}

/* the period change d of the next period; spread spectrum: on top of the table's length for that period, which
 * the DMA burst of this update writes (entry idx of a `words`-wide table, DMA channel count at cntr) */
static int32_t slew_next(volatile uint16_t *cnt, uint32_t tgt, int32_t pend, const uint16_t *tab, uint32_t words,
                         const volatile uint32_t *cntr, volatile uint16_t *atrlr, uint32_t *idx_out)
{
    int32_t d;
    if (!cur_spread) {
        d = stagger_slew(phase_of(*cnt), tgt, pend, cur_steps, slew_step_max(), cur_steps + slew_step_max() <= 65536u);
        *atrlr = (uint16_t)((int32_t)cur_steps - 1 + d);
        *idx_out = 0xFFFFFFFFu;
        return d;
    }
    {
        uint32_t idx, arr;
        d = stagger_slew_straight(phase_straight(*cnt), tgt, pend, slew_step_max());
        idx = slew_entry(cnt, cntr, words);
        arr = tab[words * idx];
        if ((int32_t)arr + d > 65535) d = 65535 - (int32_t)arr;
        *atrlr = (uint16_t)((int32_t)arr + d);
        *idx_out = idx;
    }
    return d;
}

void slew2_isr(void);                                    /* TIM2 update, entered via isr.S */
void slew2_isr(void)
{
    PERF_T(pt);                                          /* (a PERF build: core/perf.h) */
    int32_t d;
    uint32_t idx;
    TIM2->INTFR = (uint16_t)~TIM_UIF;
    d = slew_next(&TIM2->CNT, slew_tgt2, slew_pend2, pwm_tab2, 6u, &DMA1_Channel1->CNTR, &TIM2->ATRLR, &idx);
    slew_pend2 = d;
    if (d == 0) {
        TIM2->DMAINTENR &= (uint16_t)~TIM_UIE;           /* arrived */
    }
    PERF_ADD(PERF_SLEW, pt);
}

void slew3_isr(void);                                    /* TIM3 update, entered via isr.S */
void slew3_isr(void)
{
    PERF_T(pt);
    int32_t d;
    uint32_t idx;
    TIM3->INTFR = (uint16_t)~TIM_UIF;
    d = slew_next(&TIM3->CNT, slew_tgt3, slew_pend3, pwm_tab3, 6u, &DMA1_Channel3->CNTR, &TIM3->ATRLR, &idx);
    if (d != 0 && pwm_right_aligned(CH_W2)) {
        /* the update burst wrote the next period's CCR3 / CCR4 from table entry idx: move W2's start along */
        int32_t ccr;
        if (idx == 0xFFFFFFFFu) {
            idx = slew_entry(&TIM3->CNT, &DMA1_Channel3->CNTR, 2u);
            ccr = (int32_t)pwm_tab3[2u * idx];
        } else {
            ccr = (int32_t)pwm_tab3[6u * idx + 4u];
        }
        if (ccr != 0) {                                  /* 0 = on all period, stays */
            ccr += d;
            TIM3->CH3CVR = (uint16_t)(ccr < 0 ? 0 : ccr);
        }
    }
    slew_pend3 = d;
    if (d == 0) {
        TIM3->DMAINTENR &= (uint16_t)~TIM_UIE;
    }
    PERF_ADD(PERF_SLEW, pt);
}

uint8_t pwm_right_aligned(uint8_t ch)
{
    /* G (TIM1) and W2 (TIM3) right-aligned when staggering: they turn off where R / W1 turn on */
    return (uint8_t)(cur_stagger && cur_steps <= 65535u && (ch == CH_G || ch == CH_W2));
}

const uint16_t *pwm_spread_arr(void)
{
    return cur_spread ? spread_arr : 0;
}

uint16_t *pwm_table(uint8_t ch, uint8_t *stride)
{
    if (cur_spread) {
        switch (ch) {
        case CH_G: *stride = 4; return &pwm_tab1[2];
        case CH_R: *stride = 4; return &pwm_tab1[3];
        case CH_B: *stride = 6; return &pwm_tab2[5];
        case CH_W2: *stride = 6; return &pwm_tab3[4];
        default: *stride = 6; return &pwm_tab3[5];
        }
    }
    switch (ch) {
    case CH_G: *stride = 2; return &pwm_tab1[0];
    case CH_R: *stride = 2; return &pwm_tab1[1];
    case CH_B: *stride = 1; return &pwm_tab2[0];
    case CH_W2: *stride = 2; return &pwm_tab3[0];
    default: *stride = 2; return &pwm_tab3[1];
    }
}

/* Restart must never temporarily undo an overcurrent force-off. The poll ISR can
 * trip between timer writes, so sample pwm_forced and install each mode atomically. */
static void pwm_config_modes(volatile uint16_t *reg, uint16_t value, uint16_t mask)
{
    __disable_irq();
    if (pwm_forced) {
        value = (uint16_t)((value & ~mask) | ((OCM_FORCE_LO * 0x1010u) & mask));
    }
    *reg = value;
    __enable_irq();
}

void pwm_start(uint32_t steps, uint8_t len, uint8_t stagger)
{
    uint32_t mg, mw2;

    /* phase slew off first (back at the fixed offsets below): a slew ISR still due runs now, on the old cur_* and
     * running timers (with TIM2 stopped, its wait for count 24 would never end) */
    TIM2->DMAINTENR = 0;
    TIM3->DMAINTENR = 0;
    (void)TIM3->DMAINTENR;                               /* the writes are done: no new request */
    BARRIER();
    cur_steps = steps;
    cur_stagger = stagger;
    cur_len = len;
    cur_spread = (uint8_t)(stagger_is_spread(stagger) && len >= 2u);
    mg = pwm_right_aligned(CH_G) ? OCM_PWM2 : OCM_PWM1;
    mw2 = pwm_right_aligned(CH_W2) ? OCM_PWM2 : OCM_PWM1;

    TIM1->CTLR1 = 0;
    TIM2->CTLR1 = 0;
    slew_pend2 = slew_pend3 = 0;
    DMA1_Channel1->CFGR = 0;
    /* TIM3 is not stopped: its CH1 captures DIN for the edge counter (TIM4) */
    DMA1_Channel5->CFGR = 0;
    DMA1_Channel3->CFGR = 0;

    /* TIM1: CH1 = G (PA8), CH2 = R (PA9); CC1E / CC2E only (board.h: PA10 / PB14 / PB15 stay GPIO) */
    TIM1->PSC = 0;
    TIM1->ATRLR = (uint16_t)(steps - 1u);
    TIM1->RPTCR = 0;
    pwm_config_modes(&TIM1->CHCTLR1, (uint16_t)((mg << 4) | TIM_OC1PE | (OCM_PWM1 << 12) | TIM_OC2PE),
                     (7u << 4) | (7u << 12));
    TIM1->BDTR = TIM_MOE;                                /* no dead time, no break */
    TIM1->CCER = TIM_CC1E | TIM_CC2E;
    if (cur_spread) {
        uint32_t k;
        /* period table, and the fixed words of the bursts (the CCR words come from dither_build_var) */
        stagger_spread_arr(steps, len, spread_arr);
        for (k = 0; k < len; k++) {
            pwm_tab1[4u * k] = spread_arr[k];
            pwm_tab1[4u * k + 1u] = 0;                   /* RPTCR */
            pwm_tab2[6u * k] = spread_arr[k];
            pwm_tab2[6u * k + 1u] = pwm_tab2[6u * k + 2u] = pwm_tab2[6u * k + 3u] = 0;
            pwm_tab2[6u * k + 4u] = 2u;                  /* CCR3: the DMA trigger stays at count 2 */
            pwm_tab3[6u * k] = spread_arr[k];
            pwm_tab3[6u * k + 1u] = pwm_tab3[6u * k + 2u] = pwm_tab3[6u * k + 3u] = 0;
        }
        TIM1->DMACFGR = (uint16_t)((3u << 8) | (TIM_OFS_ATRLR / 4u));   /* burst of 4: ARR, RPT, CCR1, CCR2 */
        DMA1_Channel5->CNTR = (uint32_t)len * 4u;
    } else {
        TIM1->DMACFGR = (uint16_t)((1u << 8) | (TIM_OFS_CH1CVR / 4u));   /* burst of 2: CCR1, CCR2 */
        DMA1_Channel5->CNTR = (uint32_t)len * 2u;
    }
    /* every channel's table for the new period and mode, from the values the tables held (main.c; at boot 0: off),
     * while the three DMA channels are off (0.6.4, gpt-hw H2: 0.6.3 started on the old tables until main.c's
     * rebuild, and at boot on zero tables: G / W2, right-aligned when staggering (PWM mode 2, active while
     * CNT >= CCR), full on, through the boot's flash erase too) */
    pwm_fill();
    TIM1->DMAINTENR = TIM_UDE;
    DMA1_Channel5->PADDR = (uint32_t)&TIM1->DMAADR;      /* TIM1_UP = DMA1 channel 5 */
    DMA1_Channel5->MADDR = (uint32_t)pwm_tab1;
    DMA1_Channel5->CFGR = DMA_MEM2PER_16_CIRC | DMA_CFGR1_PL_1 | DMA_CFGR1_EN;

    /* TIM2 (partial remap 2): CH4 = B (PB11). TIM2_UP shares DMA1 channel 2 with the DIN sampler, so CH3 (no
     * output) compares at count 2 of every period and requests DMA1 channel 1, which writes the next CCR4
     * (preloaded: used from the next period, like the update bursts of TIM1 / TIM3). */
    TIM2->PSC = 0;
    TIM2->ATRLR = (uint16_t)(steps - 1u);
    pwm_config_modes(&TIM2->CHCTLR2, (uint16_t)((OCM_FROZEN << 4) | (OCM_PWM1 << 12) | TIM_OC4PE), 7u << 12);
    TIM2->CH3CVR = 2u;
    TIM2->CCER = TIM_CC4E;                               /* only CH4: CC1E / CC2E / CC3E = 0 (board.h invariants) */
    TIM2->DMAINTENR = TIM_CC3DE_;
    if (cur_spread) {
        /* burst of 6 from ARR on the CH3 request (the burst mechanism works on any request of the timer) */
        TIM2->DMACFGR = (uint16_t)((5u << 8) | (TIM_OFS_ATRLR / 4u));
        DMA1_Channel1->PADDR = (uint32_t)&TIM2->DMAADR;
        DMA1_Channel1->CNTR = (uint32_t)len * 6u;
    } else {
        DMA1_Channel1->PADDR = (uint32_t)&TIM2->CH4CVR;  /* TIM2_CH3 = DMA1 channel 1 */
        DMA1_Channel1->CNTR = len;
    }
    DMA1_Channel1->MADDR = (uint32_t)pwm_tab2;
    DMA1_Channel1->CFGR = DMA_MEM2PER_16_CIRC | DMA_CFGR1_PL_1 | DMA_CFGR1_EN;

    /* TIM3: CH3 = W2 (PB0), CH4 = W1 (PB1); CH1 = DIN capture, kept. CC2E stays 0 (PA7 is the current sense) */
    TIM3->PSC = 0;
    TIM3->ATRLR = (uint16_t)(steps - 1u);
    pwm_config_modes(&TIM3->CHCTLR2, (uint16_t)((mw2 << 4) | TIM_OC3PE | (OCM_PWM1 << 12) | TIM_OC4PE),
                     (7u << 4) | (7u << 12));
    TIM3->CCER = TIM_CC1E | TIM_CC1P | TIM_CC3E | TIM_CC4E;
    if (cur_spread) {
        TIM3->DMACFGR = (uint16_t)((5u << 8) | (TIM_OFS_ATRLR / 4u));   /* burst of 6: ARR .. CCR4 */
        DMA1_Channel3->CNTR = (uint32_t)len * 6u;
    } else {
        TIM3->DMACFGR = (uint16_t)((1u << 8) | (TIM_OFS_CH3CVR / 4u));   /* burst of 2: CCR3, CCR4 */
        DMA1_Channel3->CNTR = (uint32_t)len * 2u;
    }
    TIM3->DMAINTENR = TIM_UDE;
    DMA1_Channel3->PADDR = (uint32_t)&TIM3->DMAADR;      /* TIM3_UP = DMA1 channel 3 */
    DMA1_Channel3->MADDR = (uint32_t)pwm_tab3;
    DMA1_Channel3->CFGR = DMA_MEM2PER_16_CIRC | DMA_CFGR1_PL_1 | DMA_CFGR1_EN;

    /* the first period's compares (UG makes the preloads active; its burst brings the next period's): entry 0 of
     * each channel's table (pwm_table: G, R at tab1 [0], [1]; B tab2 [0]; W2, W1 tab3 [0], [1]; spread: +2, +5, +4) */
    {
        uint32_t s1 = cur_spread ? 2u : 0u, s3 = cur_spread ? 4u : 0u;
        int32_t g = pwm_tab1[s1], w2 = pwm_tab3[s3];
        if (cur_spread) {
            /* the first period is `steps` long, entry 0's period is spread_arr[0] + 1: a right-aligned compare (the
             * period - the pulse) moves by the difference (an "off" G / W2 would show the difference as a pulse) */
            int32_t k = (int32_t)steps - (int32_t)spread_arr[0] - 1;
            g += mg == OCM_PWM2 ? k : 0;
            w2 += mw2 == OCM_PWM2 ? k : 0;
        }
        TIM1->CH1CVR = (uint16_t)(g < 0 ? 0 : g);
        TIM1->CH2CVR = pwm_tab1[s1 + 1u];
        TIM2->CH4CVR = pwm_tab2[cur_spread ? 5u : 0u];
        TIM3->CH3CVR = (uint16_t)(w2 < 0 ? 0 : w2);
        TIM3->CH4CVR = pwm_tab3[s3 + 1u];
    }
    /* load ARR / first bursts, then the phases: a timer started at count steps - x wraps (its R / B / W1 turn-on,
     * its G / W2 turn-off) x counts after TIM1 */
    TIM1->SWEVGR = TIM_UG;
    TIM3->SWEVGR = TIM_UG;
    TIM2->SWEVGR = TIM_UG;
    TIM2->CNT = (uint16_t)(stagger ? steps - steps / 3u : 0u);           /* B at 1/3 of the period */
    TIM3->CNT = (uint16_t)(stagger ? steps - (2u * steps) / 3u : 0u);    /* W1 / W2 at 2/3 */
    TIM1->CNT = 0;
    if (cur_spread) {
        /* all three walk the same period sequence: each timer's first table period must be entry 0. TIM1 wraps at
         * once (its update burst after UG holds entry 0); TIM3's UG burst likewise; TIM2's burst comes on CH3, so
         * its first compare is set one count after its start, before its first wrap (the burst resets CCR3 to 2) */
        TIM1->CNT = (uint16_t)(steps - 1u);
        TIM2->CH3CVR = (uint16_t)(steps - steps / 3u + 1u);
    }
    TIM1->CTLR1 = TIM_ARPE | TIM_CEN;
    TIM2->CTLR1 = TIM_ARPE | TIM_CEN;
    TIM3->CTLR1 = TIM_ARPE | TIM_CEN;
    if (pwm_forced) {
        pwm_outputs_mode();                              /* a trip (poll tick) during the restart stays in force */
    }
}

/* ---------------------------------------------------------------------------------------------------- isense */
/* OCxM of the five PWM channels: force inactive (pin low: driver input low, FET off) or the PWM modes of
 * pwm_start. OCxM is not preloaded, so it acts at once; the DMA keeps loading CCRs meanwhile. Read-modify-write
 * of the channel's 3 OCxM bits only: TIM2 CH3 (B's DMA trigger, frozen) and TIM3 CH1 (DIN capture) stay. */
static void pwm_outputs_mode(void)
{
    uint32_t f = pwm_forced;
    uint32_t mg = f ? OCM_FORCE_LO : (pwm_right_aligned(CH_G) ? OCM_PWM2 : OCM_PWM1);
    uint32_t mw2 = f ? OCM_FORCE_LO : (pwm_right_aligned(CH_W2) ? OCM_PWM2 : OCM_PWM1);
    uint32_t m = f ? OCM_FORCE_LO : OCM_PWM1;
    TIM1->CHCTLR1 = (uint16_t)((TIM1->CHCTLR1 & ~((7u << 4) | (7u << 12))) | (mg << 4) | (m << 12));
    TIM2->CHCTLR2 = (uint16_t)((TIM2->CHCTLR2 & ~(7u << 12)) | (m << 12));
    TIM3->CHCTLR2 = (uint16_t)((TIM3->CHCTLR2 & ~((7u << 4) | (7u << 12))) | (mw2 << 4) | (m << 12));
}

void pwm_force_off(uint8_t off)
{
    pwm_forced = off;
    pwm_outputs_mode();
}

/* ---------------------------------------------------------------------------------------------------- ADC */
/* ADC1 at 12 MHz, no DMA (all seven DMA1 channels are taken).
 * Regular group: PA7 = ADC_IN7, the current sense, converted continuously: 71.5 cycles (6 us, any source up to 50 k),
 * 84 cycles = 7 us per result; ISENSE_RAW() is the newest.
 * Injected group, started by the poll tick once per ms (hw_adc_ms):
 *  - the voltage channels: IN4, IN0, IN4, IN0 (PA4 the supply, PA0 TP8), 71.5 cycles each (28 us);
 *  - instead, once a second when hw_temp_start() asked: the temperature sensor (IN16) and VREFINT (IN17), 16, 17,
 *    16, 17 at 239.5 cycles (20 us each, the datasheet asks >= 17.1 us; 84 us).
 * Either way 1 + 3 and 2 + 4 are the pairs, whichever way IDATAR1-4 map to the ranks. Scan mode (the regular group is
 * one channel), so the whole group converts; a running regular conversion is reset and resumes afterwards: the poll
 * tick reads a stale PA7 value for up to 28 us per ms (84 us once a second), so a short in that moment trips that
 * much later. A sequence is read at the next ms (it ends well within it). BENCH: the start sequence (RM 12.2.2 /
 * 12.2.4: ADON again starts, CONT keeps converting) and the injected insertion during continuous conversion. */
#define INJ_SEQ(a, b) ((3u << 20) | ((uint32_t)(b) << 15) | ((uint32_t)(a) << 10) | ((uint32_t)(b) << 5) | (uint32_t)(a))
#define INJ_VOLT      INJ_SEQ(VSENSE_SUPPLY_CH, 0u)
#define INJ_TEMP      INJ_SEQ(16u, 17u)

static volatile uint8_t inj_run;                         /* the sequence running: 0 none, 1 voltages, 2 temperature */
static volatile uint8_t temp_req, temp_done;
static volatile uint32_t temp_ts2, temp_vref2;
static volatile uint32_t vs_acc[2], vs_n;                /* voltage pairs since the main loop's last take */
volatile uint8_t adc_ready;                              /* hw_adc_init done: the poll tick may start sequences (0
                                                            during flash_op too, hw_priv.h: the SRAM tick does not run them) */
volatile uint16_t hw_diag[HW_DIAG_N];                    /* (0.6.5, D6) hw.h */

void hw_adc_init(void)
{
    uint32_t i;
    RCC->APB2PCENR |= RCC_ADC1EN;
    ADC1->CTLR1 = ADC_SCAN;                              /* independent; scan: the injected group converts in full */
    ADC1->SAMPTR1 = (7u << 18) | (7u << 21);             /* SMP16, SMP17 = 111: 239.5 cycles */
    ADC1->SAMPTR2 = (6u << 21) | (6u << (3u * VSENSE_SUPPLY_CH)) | 6u; /* SMP7, SMP4, SMP0: 71.5 cycles */
    ADC1->RSQR1 = 0;                                     /* L = 0: one regular conversion */
    ADC1->RSQR3 = 7u;                                    /* SQ1 = channel 7 */
    ADC1->ISQR = INJ_VOLT;
    ADC1->CTLR2 = ADC_ADON | ADC_JEXTTRIG | ADC_JEXTSEL; /* calibrate with TSVREFE/BUFEN clear (RM 12.3.2) */
    for (i = 0; i < 1000u; i++) {                        /* tSTAB 1 us (and >= 2 ADC clocks before calibration) */
        __asm volatile("nop");
    }
    ADC1->CTLR2 |= ADC_RSTCAL;
    for (i = 0; i < 100000u && (ADC1->CTLR2 & ADC_RSTCAL); i++) {
    }
    ADC1->CTLR2 |= ADC_CAL;
    for (i = 0; i < 100000u && (ADC1->CTLR2 & ADC_CAL); i++) {
    }
    ADC1->CTLR2 |= ADC_TSVREFE;                          /* enabling it forces the buffer on: only after CAL */
    ADC1->CTLR2 |= ADC_CONT;
    ADC1->CTLR2 |= ADC_ADON;                             /* ADON again: start; CONT: keep converting */
    adc_ready = 1;
}

uint32_t hw_adc_ms(void)
{
    uint32_t sr;
    if (!adc_ready) {
        return 0;
    }
    sr = ADC1->STATR;
#if MON_EXTRA
    if (!(sr & ADC_STRT)) {
        hw_diag[0]++;                    /* (0.6.5, D6) no regular conversion started since the last ms (STRT, cleared
                                            below: BENCH, continuous mode sets it at each start) */
    }
#endif
    if (inj_run != 0u && (sr & ADC_JEOC)) {
        uint32_t a = (ADC1->IDATAR1 & 0xFFFu) + (ADC1->IDATAR3 & 0xFFFu);
        uint32_t b = (ADC1->IDATAR2 & 0xFFFu) + (ADC1->IDATAR4 & 0xFFFu);
        if (inj_run == 2u) {
            temp_ts2 = a;
            temp_vref2 = b;
            temp_done = 1;
        } else {
            vs_acc[0] += a;
            vs_acc[1] += b;
            vs_n++;
        }
#if MON_EXTRA
    } else if (inj_run != 0u) {
        hw_diag[1]++;                    /* (0.6.5, D6) the last sequence is not done: its values are not taken */
#endif
    }
    ADC1->STATR = ~(uint32_t)(ADC_JEOC | (MON_EXTRA ? ADC_STRT : 0u));   /* rc_w0: only JEOC (and STRT) cleared */
    if (temp_req) {
        temp_req = 0;
        ADC1->ISQR = INJ_TEMP;
        inj_run = 2;
    } else {
        ADC1->ISQR = INJ_VOLT;
        inj_run = 1;
    }
    ADC1->CTLR2 |= ADC_JSWSTART;
    return inj_run - 1u;                                 /* 1: the temperature's 84 us (main.c: the next tick) */
}

void hw_vsense_take(uint32_t sum2[2], uint32_t *n)
{
    __disable_irq();
    sum2[0] = vs_acc[0];
    sum2[1] = vs_acc[1];
    *n = vs_n;
    vs_acc[0] = vs_acc[1] = vs_n = 0;
    __enable_irq();
}

void hw_temp_start(void)
{
    temp_done = 0;
    temp_req = 1;                                        /* converted at the poll tick's next ms */
}

uint8_t hw_temp_read(uint32_t *ts2, uint32_t *vref2)
{
    if (!temp_done) {
        return 0;
    }
    *ts2 = temp_ts2;
    *vref2 = temp_vref2;
    return 1;
}

/* (0.6.5, D6) DMA1's transfer error flags (TEIF1..7, RM 10.4.1): counted, and cleared (only the ones read) */
void hw_dma_check(void)
{
    uint32_t f = DMA1->INTFR & 0x08888888u, i;
    for (i = 0; i < 7u; i++) {
        if (f & (DMA_TEIF1 << (4u * i))) {
            hw_diag[2u + i]++;
        }
    }
    DMA1->INTFCR = f;                                    /* (CTEIFx at the TEIFx positions) */
}

/* ---------------------------------------------------------------------------------------------------- gate */
static volatile uint16_t gate_t1, gate_t2;

/* TIM3 CH1 captures every DIN falling edge on PA6 (filter N = 2, the pin the SPI sampler reads too); its trigger
 * output pulses on each capture (MMS = 011) and clocks TIM4 (ITR2, external clock mode 1), so TIM4->CNT counts
 * DIN edges. TIM4 has no pins: at CNT == CCR2 its CC2 DMA request makes DMA1 channel 4 write GPIOA->BSHR, which
 * sets or clears PA1 (FWD_EN) a few 10 ns after the edge, well inside the bit's low time (>= 580 ns).
 * The CC2 interrupt then moves CCR2 to t2 and loads the opposite write (WS2915 / Ultra), or ends.
 * BENCH: edge -> PA1 delay (expected 80-150 ns), DMA access to GPIOA. */
static volatile uint32_t gate_word;                      /* BSHR value of the next toggle */

static void gate_arm(uint16_t at, uint32_t word)
{
    gate_word = word;
    DMA1_Channel4->CFGR = 0;
    DMA1_Channel4->MADDR = (uint32_t)&gate_word;
    DMA1_Channel4->CNTR = 1;
    DMA1_Channel4->CFGR = DMA_CFGR1_DIR | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PL | DMA_CFGR1_EN;
    TIM4->CH2CVR = at;
}

void gate_force_closed(void)
{
    TIM4->DMAINTENR = 0;
    DMA1_Channel4->CFGR = 0;
    gate_t1 = 0;
    gate_t2 = 0;
    GPIOA->BSHR = 1u << (FWD_PIN + 16u);
}

NOINLINE void gate_rearm(const gate_plan_t *g)                 /* (poll ISR, frame end: lto.h) */
{
    TIM4->DMAINTENR = 0;
    TIM4->CNT = 0;
    GPIOA->BSHR = g->start_open ? (1u << FWD_PIN) : (1u << (FWD_PIN + 16u));
    gate_t1 = g->t1;
    gate_t2 = g->t2;
    TIM4->INTFR = (uint16_t)~TIM_CC2IF;
    if (g->t1 != 0u) {
        gate_arm(g->t1, g->start_open ? (1u << (FWD_PIN + 16u)) : (1u << FWD_PIN));
        TIM4->DMAINTENR = TIM_CC2DE_ | TIM_CC2IE_;
    } else {
        DMA1_Channel4->CFGR = 0;
    }
}

void gate_isr(void);                                     /* TIM4 CC2, entered via isr.S (HPE + mret) */
void gate_isr(void)
{
    PERF_T(pt);
    uint32_t n = 0;
    TIM4->INTFR = (uint16_t)~TIM_CC2IF;
    if (gate_t2 != 0u && TIM4->CH2CVR == gate_t1) {
        while (DMA1_Channel4->CNTR != 0u && ++n < 64u) {
            /* the first toggle is written a few cycles after the match */
        }
        gate_arm(gate_t2, (GPIOA->OUTDR & (1u << FWD_PIN)) ? (1u << (FWD_PIN + 16u)) : (1u << FWD_PIN));
    } else {
        TIM4->DMAINTENR = 0;                             /* hold the level until the next re-arm */
    }
    PERF_ADD(PERF_GATE, pt);
}

uint32_t din_ring_pos(void)
{
    return (DIN_RING_WORDS - DMA1_Channel2->CNTR) & (DIN_RING_WORDS - 1u);
}

/* ---------------------------------------------------------------------------------------------------- misc */

uint32_t hw_uid_hash(void)
{
    const volatile uint32_t *u = (const volatile uint32_t *)0x1FFFF7E8u;   /* RM 31.2: UID registers 1..3 */
    uint32_t h = (u[0] ^ u[1] ^ u[2]) * 0x9E3779B9u;
    return h ^ (h >> 16);
}

void led_set(uint8_t on)
{
    GPIOB->BSHR = on ? (1u << 15) : (1u << (15 + 16));  /* D9 on PB15 */
}

#if BOARD_RETURN
/* ------------------------------------------------------------------------------------------- return channel */
/* USART3 TX only (RE 0: its RX pin PB11 is PWM_B), 8N1, by interrupt: every DMA channel is taken (USART3_TX's, ch2,
 * is the DIN sampler's). Its entry: VTF slot 1 (0.6.5's unified trap_entry dispatches only SysTick and TIM2-4;
 * slot 0 is flash_op's), priority 0x00, so it preempts the poll tick (a byte every 10 us at 1 Mbaud; a late one only
 * stretches the stop bit). PB10 is driven only during a burst; otherwise an input with its weak pull-down, and the relay node
 * follows the output transceiver's RO (1 k against the 100 R from PB10). BENCH: the node's levels, DIN's read-back. */
static const uint8_t *rc_next;
static uint32_t rc_left;
typedef char rc_one_lead_byte[RC_LEAD_BYTES == 1u ? 1 : -1];   /* (rc_hw_send writes one 0xFF) */

static void rc_release(void)
{
    USART3->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE;     /* no TXE / TC interrupt */
    pin_cfg(GPIOB, 10, CFG_IN_PULL);
}

void rc_hw_dir(uint32_t mode)
{
    if (mode == RC_DIR_FORWARD) {
        rc_release();
        GPIOB->BCR = 1u << 14;                           /* input transceiver: listen (DE 0) */
        GPIOA->BSHR = 1u << 10;                          /* output transceiver: drive downstream (DE 1) */
    } else {
        GPIOA->BCR = 1u << 10;                           /* output transceiver: listen */
        GPIOB->BSHR = mode == RC_DIR_ANSWER ? 1u << 14 : 1u << (14 + 16);   /* input: drive upstream from the node,
                                                                                or listen too (RC_DIR_QUIET) */
    }
}

uint32_t rc_hw_send(const uint8_t *b, uint32_t n, uint16_t brr)
{
    uint32_t t = (uint32_t)SysTick->CNT, low;            /* (early in the tick: no reload during the wait) */
    USART3->BRR = brr;
    pin_cfg(GPIOB, 10, CFG_AF_PP);                       /* the line HIGH: the USART idles high */
    while ((uint32_t)SysTick->CNT - t < RC_PREAMBLE_CYC) {
    }
    low = (GPIOA->INDR & (1u << 6)) == 0u;               /* DIN (the input RO) LOW: another driver on the segment */
    if (low || n == 0u) {                                /* (n 0: the turnaround check only) */
        rc_release();
        return low;
    }
    rc_next = b;
    rc_left = n;
    BARRIER();                                           /* (r6) the cursor published before its interrupt */
    USART3->STATR = (uint16_t)~USART_STATR_TC;
    USART3->DATAR = 0xFFu;                               /* RC_LEAD_BYTES (core/rc.h): the hub's UART in step */
    USART3->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE | USART_CTLR1_TXEIE;
    return 0;
}

void rc_uart_isr(void);                                  /* USART3, entered via isr.S (VTF, HPE + mret) */
void rc_uart_isr(void)
{
    BARRIER();                                           /* acquire the cursor published by the poll ISR */
    if (USART3->CTLR1 & USART_CTLR1_TXEIE) {
        if (rc_left != 0u) {
            rc_left--;
            USART3->DATAR = *rc_next++;
            return;
        }
        USART3->STATR = (uint16_t)~USART_STATR_TC;       /* the last byte is in the shift register: its stop bit */
        USART3->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE | USART_CTLR1_TCIE;
        return;
    }
    rc_release();                                        /* TC: the line back to the relay node (LOW idle) */
}
#endif

/* the stack (Link_sections.ld: _susrstack .. _eusrstack, 2 KB at the top of RAM; tools/build_target.sh STACK_SIZE):
 * painted below the caller's frame at boot; the lowest word no longer holding the pattern is the deepest the stack (main loop +
 * nested ISRs) went */
#define STACK_PAINT 0xC5A3965Au
extern uint32_t _susrstack[], _eusrstack[];

void stack_paint(void)
{
    uint32_t *p = _susrstack, *sp;
    __asm volatile("mv %0, sp" : "=r"(sp));
    for (sp -= 8; p < sp; p++) {                         /* (this frame's words stay) */
        *p = STACK_PAINT;
    }
}

uint32_t stack_used(void)
{
    const uint32_t *p = _susrstack;
    while (p < _eusrstack && *p == STACK_PAINT) {
        p++;
    }
    return (uint32_t)((const uint8_t *)_eusrstack - (const uint8_t *)p);
}

/* the independent watchdog (hw_cold.c watchdog_start: LSI / 32, reload 500, a reset after 0.27-0.64 s without a
 * kick). The main loop kicks every pass; its longest stall is a settings write with a page erase (~18 ms). */
void watchdog_kick(void)
{
    IWDG->CTLR = 0xAAAAu;
}

/* ---------------------------------------------------------------------------------------------------- UART */
/* USART2 115200 8N1. TX: DMA1 ch7 from one of two buffers: uart_write fills the idle buffer, uart_kick sends it.
 * Blocks only when a buffer's worth is pending while one is in flight. (0.6.7) The release console (core/config.h
 * CONSOLE_FULL 0) only sends: two buffers of 256 (each part it writes fits both: main.c waits for CLI_PART_ROOM), no
 * RX (DMA1 ch6 free; PA3 / TP6 the test jig's pad alone). CONSOLE_FULL 1: RX by DMA1 ch6 circular into uart_rx,
 * two buffers of 512. */
#if CONSOLE_FULL
#define UART_TX_SIZE 512u
uint8_t uart_rx[UART_RX_SIZE];                           /* (hw_priv.h: hw_init starts the RX DMA into it) */
uint32_t uart_rx_r;                                      /* (hw_priv.h: flash_op drops what came at the halved baud) */
uint8_t uart_rx_drop;                                    /* flash_op dropped bytes: 1 = tell main.c, 2 = to CR / LF */
#else
#define UART_TX_SIZE 256u
#endif
static uint8_t uart_tx[2][UART_TX_SIZE];
static uint32_t uart_tx_len;
static uint8_t uart_tx_fill;

#if CONSOLE_FULL
uint32_t uart_rx_w(void)
{
    return (UART_RX_SIZE - DMA1_Channel6->CNTR) % UART_RX_SIZE;
}

uint32_t uart_rx_pos(void)
{
    return uart_rx_w();
}

int uart_getc(void)
{
    uint32_t w = uart_rx_w();
    int c;
    if (uart_rx_drop == 1u) {
        uart_rx_drop = 2;
        return -2;                                       /* (main.c: the line being typed is cancelled) */
    }
    if (uart_rx_r == w) {
        return -1;
    }
    c = uart_rx[uart_rx_r];
    uart_rx_r = (uart_rx_r + 1u) % UART_RX_SIZE;
    if (uart_rx_drop) {
        if (c == '\r' || c == '\n') {
            uart_rx_drop = 0;                            /* the rest of the cut line is gone, the next one counts */
        }
        return -1;
    }
    return c;
}
#endif

static int uart_tx_busy(void)
{
    return (DMA1_Channel7->CFGR & DMA_CFGR1_EN) && DMA1_Channel7->CNTR != 0u;
}

uint32_t uart_room(void)
{
    return UART_TX_SIZE - uart_tx_len + (uart_tx_busy() ? 0u : UART_TX_SIZE);
}

uint32_t uart_busy(void)
{
    return uart_tx_len != 0u || uart_tx_busy() || !(USART2->STATR & USART_STATR_TC);
}

/* the DMA start (out of line: uart_kick's every-pass test stays a leaf without a frame) */
static NOINLINE void uart_kick_tx(void)
{
    DMA1_Channel7->CFGR = 0;
    DMA1_Channel7->MADDR = (uint32_t)uart_tx[uart_tx_fill];
    DMA1_Channel7->CNTR = uart_tx_len;
    DMA1_Channel7->CFGR = DMA_CFGR1_DIR | DMA_CFGR1_MINC | DMA_CFGR1_EN;
    uart_tx_fill ^= 1u;
    uart_tx_len = 0;
}

void uart_kick(void)
{
    if (uart_tx_len != 0u && !uart_tx_busy()) {
        uart_kick_tx();
    }
}

void uart_write(const char *s)
{
    while (*s) {
        if (uart_tx_len == UART_TX_SIZE) {
            while (uart_tx_busy()) {
            }
            uart_kick();
        }
        uart_tx[uart_tx_fill][uart_tx_len++] = (uint8_t)*s++;
    }
}

/* (0.6.5) the record reads at boot (main.c, before hw_init: no timer, DMA, UART or SysTick runs on the clocks yet).
 * Upper layout: HCLK / 2, so the flash clock (HCLK / 2 with SCKMOD 0, its reset value, which nothing here changes) is
 * 36 MHz, inside DS table 4-17's 60 MHz for reads of the non-zero-wait CodeFlash (README: Upper flash;
 * the zero-wait area's 144 MHz is for the code path, not for these reads). The core runs at 72 MHz meanwhile. */
void hw_records_window(uint32_t open)
{
#if UPPER_RECORDS
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | (open ? RCC_HPRE_DIV2 : 0u);
#else
    (void)open;                                          /* 0.6.4's layout: zero-wait flash, read at any clock */
#endif
}

uint32_t hw_vdd_low(void)
{
    return (PWR->CSR & PWR_CSR_PVDO) != 0u;             /* PVDO = 1: VDD below the threshold (flash_op's test) */
}

uint32_t hw_boot_open(void)
{
    return (FLASH->WPR & 3u) != 0u;                      /* a bit at 1: that 4 KB sector is writable (RM 32.4.7) */
}

/* (0.6.9, review round 7 R7-06) RM 32.1 note 2 counts the user option reads among the flash accesses that need HCLK
 * divided above 120 MHz: at 144 MHz with SCKMOD 0 the flash clock is 72 MHz, over the datasheet's 60. So HCLK / 2
 * meanwhile (36 MHz), as the records window, which is why main.c calls this before hw_init (no timer, interrupt, DMA
 * or UART runs on the clocks yet). Each half-word: a byte and its inverse (RDPR / nRDPR ...), each read twice */
uint32_t hw_option_bytes(uint16_t ob[8])
{
    const volatile uint16_t *o = (const volatile uint16_t *)HW_OB_ADDR;
    uint32_t cfgr = RCC->CFGR0, i, ok = 1, a;
    RCC->CFGR0 = (cfgr & ~RCC_HPRE) | RCC_HPRE_DIV2;
    for (i = 0; i < 8u; i++) {
        a = o[i];
        ok &= a == o[i] && ((a ^ (a >> 8)) & 0xFFu) == 0xFFu;
        ob[i] = (uint16_t)(a & 0xFFu);
    }
    RCC->CFGR0 = cfgr;
    return ok;
}

/* the SRAM path's trip: all five outputs forced low (as pwm_force_off(1)); never back to PWM */
RAMCODE void hw_flash_trip(void)
{
    PWM_TRIP();
}

