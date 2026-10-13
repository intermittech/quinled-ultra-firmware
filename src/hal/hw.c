/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V203F8U6 drivers: clock, GPIO, PWM timers + dither DMA, DIN edge counter / gate, DIN sampler, flash.
 * Register usage follows the WCH CH32FV2x_V3xRM reference manual V2.5. Items marked BENCH need confirmation on real
 * hardware. */
#include "ch32v20x.h"
#include "hw.h"
#include "../core/store.h"
#include "../core/stagger.h"
#include "../core/scratch.h"
#include "../core/lto.h"

uint32_t SystemCoreClock = 144000000u;

/* DMA tables. Plain: TIM1 {CCR1, CCR2(, CCR3)}, TIM2 {CCR4}, TIM3 {CCR3, CCR4} per entry. Spread spectrum: every
 * entry also sets the period, burst from ATRLR: TIM1 {ARR, RPT, CCR1, CCR2}, TIM2 {ARR, -, CCR1, CCR2, CCR3 = 2,
 * CCR4}, TIM3 {ARR, -, CCR1 (capture: read-only), CCR2, CCR3, CCR4}. */
uint16_t pwm_tab1[PATTERN_MAX_LEN * 4];
uint16_t pwm_tab2[PATTERN_MAX_LEN * 6];
uint16_t pwm_tab3[PATTERN_MAX_LEN * 6];
uint16_t din_ring[DIN_RING_WORDS] __attribute__((aligned(4)));
uint16_t din_copy[DIN_COPY_WORDS] __attribute__((aligned(4)));

static uint32_t cur_steps;
static uint8_t cur_stagger, cur_len, cur_spread;
static uint16_t spread_arr[PATTERN_MAX_LEN];         /* spread spectrum: period - 1 of each table entry */
static volatile uint8_t pwm_forced;                  /* isense trip: all PWM outputs forced low */
static void pwm_outputs_mode(void);

/* GPIO CFGxR nibbles (STM32F1 style) */
#define CFG_IN_FLOAT 0x4u
#define CFG_OUT_PP   0x3u        /* 50 MHz push-pull */
#define CFG_AF_PP    0xBu        /* 50 MHz alternate-function push-pull */
#define CFG_ANALOG   0x0u
#define CFG_IN_PULL  0x8u        /* input with pull-up / pull-down (ODR) */

/* TIMx_CHCTLRx output compare modes */
#define OCM_FROZEN   0u
#define OCM_TOGGLE   3u
#define OCM_FORCE_LO 4u
#define OCM_FORCE_HI 5u
#define OCM_PWM1     6u          /* active while CNT < CCR */
#define OCM_PWM2     7u          /* active while CNT >= CCR */

/* TIM register offsets for the DMA burst base address (DBA = offset / 4) */
#define TIM_OFS_ATRLR  0x2Cu
#define TIM_OFS_CH1CVR 0x34u
#define TIM_OFS_CH3CVR 0x3Cu

#define DMA_MEM2PER_16_CIRC (DMA_CFGR1_DIR | DMA_CFGR1_CIRC | DMA_CFGR1_MINC | DMA_CFGR1_PSIZE_0 | DMA_CFGR1_MSIZE_0)

/* TIMx DMAINTENR bits not named in the WCH header */
#define TIM_CC2IE_ 0x0004u
#define TIM_CC2DE_ 0x0400u
#define TIM_CC3DE_ 0x0800u

#define FWD_PIN    1u                                    /* PA1 = FWD_EN */

static void pin_cfg(GPIO_TypeDef *g, uint32_t pin, uint32_t cfg)
{
    volatile uint32_t *r = pin < 8u ? &g->CFGLR : &g->CFGHR;
    uint32_t sh = (pin & 7u) * 4u;
    *r = (*r & ~(0xFu << sh)) | (cfg << sh);
}

/* ---------------------------------------------------------------------------------------------------- clock */
/* HSI 8 MHz, undivided into the PLL (EXTEN HSIPRE = 1), x18 = 144 MHz. HCLK and PCLK1 undivided (allowed up to
 * 144 MHz per the datasheet), so TIM2-4 run from 144 MHz. Called by the WCH startup.
 * PCLK2 = 72 MHz in every build, because the ADC clock may not exceed 14 MHz (datasheet table 4-27; ADCPRE only
 * divides by 2 / 4 / 6 / 8): ADC = 72 / 6 = 12 MHz, for the current sense, the voltage channels and the MCU
 * temperature (with 144 MHz the ADC could only run at 18 MHz, and its 239.5-cycle sample time would be 13 us, under
 * the 17.1 us the sensor needs). TIM1 still counts at 144 MHz (RM clock tree: the APB2 timer clock is PCLK2 x 2 when
 * the APB2 prescaler is not 1); SPI1 divides by 2 (36 MHz); USART2, TIM2-4 and SysTick are not on APB2. GPIO /
 * AFIO / TIM1 / SPI1 / ADC register accesses take one 72 MHz bus cycle (~14 ns) instead of 7 ns (the DMA write of
 * the forwarding gate to GPIOA: ~+10 ns, BENCH). */
void SystemInit(void)
{
    RCC->CTLR |= RCC_HSION;
    RCC->CFGR0 &= ~(RCC_SW | RCC_HPRE | RCC_PPRE1 | RCC_PPRE2 | RCC_ADCPRE | RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL);
    EXTEN->EXTEN_CTR |= EXTEN_PLL_HSI_PRE;
    RCC->CFGR0 |= RCC_PLLSRC_HSI_Div2 | RCC_PLLMULL18;      /* "Div2" path is undivided with HSIPRE = 1 */
    RCC->CFGR0 |= RCC_PPRE2_DIV2 | RCC_ADCPRE_DIV6;
    RCC->CTLR |= RCC_PLLON;
    while ((RCC->CTLR & RCC_PLLRDY) == 0u) {
    }
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_PLL;
    while ((RCC->CFGR0 & RCC_SWS) != 0x08u) {
    }
}

/* ---------------------------------------------------------------------------------------------------- PWM */
/* Adaptive stagger: TIM2 / TIM3 move to new phase offsets (relative to TIM1) gradually. While a timer is off
 * target, its update interrupt changes the length of the next period by at most period / 512 (ARR, preloaded).
 * Only the off-time of left-aligned channels changes (< 0.2 % for that period). TIM3's right-aligned W2 ends at the
 * period end, so its CCR3 (just loaded by the update DMA burst for the same period) moves along: W2's pulse keeps
 * its width. */
static volatile uint16_t slew_tgt2, slew_tgt3;
static volatile int32_t slew_pend2, slew_pend3;

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
    uint32_t c1 = TIM1->CNT;
    return (c1 + cur_steps - (cnt_x % cur_steps)) % cur_steps;
}

/* spread spectrum: TIM1 and the timer run the same period sequence, offsets stay below 85 % of the period, so the
 * timer wrapped inside TIM1's current period: the offset is the plain difference */
static uint32_t phase_straight(uint32_t cnt_x)
{
    int32_t d = (int32_t)TIM1->CNT - (int32_t)cnt_x;
    return (uint32_t)(d < 0 ? d + (int32_t)cur_steps : d);
}

/* the period change d of the next period; spread spectrum: on top of the table's length for that period, which
 * the DMA burst of this update has just written (entry idx of a `words`-wide table at dma_cntr) */
static int32_t slew_next(volatile uint16_t *cnt, uint32_t tgt, int32_t pend, const uint16_t *tab, uint32_t words,
                         uint32_t dma_cntr, volatile uint16_t *atrlr, uint32_t *idx_out)
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
        while (*cnt < 24u) {
            /* the burst for the next period (at the update / at count 2) is written */
        }
        idx = (((uint32_t)words * cur_len - dma_cntr) / words + cur_len - 1u) % cur_len;
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
    int32_t d;
    uint32_t idx;
    TIM2->INTFR = (uint16_t)~TIM_UIF;
    d = slew_next(&TIM2->CNT, slew_tgt2, slew_pend2, pwm_tab2, 6u, DMA1_Channel1->CNTR, &TIM2->ATRLR, &idx);
    slew_pend2 = d;
    if (d == 0) {
        TIM2->DMAINTENR &= (uint16_t)~TIM_UIE;           /* arrived */
    }
}

void slew3_isr(void);                                    /* TIM3 update, entered via isr.S */
void slew3_isr(void)
{
    int32_t d;
    uint32_t idx;
    TIM3->INTFR = (uint16_t)~TIM_UIF;
    d = slew_next(&TIM3->CNT, slew_tgt3, slew_pend3, pwm_tab3, 6u, DMA1_Channel3->CNTR, &TIM3->ATRLR, &idx);
    if (d != 0 && pwm_right_aligned(CH_W2)) {
        /* the update burst wrote the next period's CCR3 / CCR4 from table entry idx: move W2's start along */
        int32_t ccr;
        while (TIM3->CNT < 24u) {
        }
        if (idx == 0xFFFFFFFFu) {
            idx = ((2u * cur_len - DMA1_Channel3->CNTR) / 2u + cur_len - 1u) % cur_len;
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
    TIM1->CHCTLR1 = (uint16_t)((mg << 4) | TIM_OC1PE | (OCM_PWM1 << 12) | TIM_OC2PE);
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
    TIM1->DMAINTENR = TIM_UDE;
    DMA1_Channel5->PADDR = (uint32_t)&TIM1->DMAADR;      /* TIM1_UP = DMA1 channel 5 */
    DMA1_Channel5->MADDR = (uint32_t)pwm_tab1;
    DMA1_Channel5->CFGR = DMA_MEM2PER_16_CIRC | DMA_CFGR1_PL_1 | DMA_CFGR1_EN;

    /* TIM2 (partial remap 2): CH4 = B (PB11). TIM2_UP shares DMA1 channel 2 with the DIN sampler, so CH3 (no
     * output) compares at count 2 of every period and requests DMA1 channel 1, which writes the next CCR4
     * (preloaded: used from the next period, like the update bursts of TIM1 / TIM3). */
    TIM2->PSC = 0;
    TIM2->ATRLR = (uint16_t)(steps - 1u);
    TIM2->CHCTLR2 = (uint16_t)((OCM_FROZEN << 4) | (OCM_PWM1 << 12) | TIM_OC4PE);
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
    TIM3->CHCTLR2 = (uint16_t)((mw2 << 4) | TIM_OC3PE | (OCM_PWM1 << 12) | TIM_OC4PE);
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
static volatile uint8_t adc_ready;                       /* hw_adc_init done: the poll tick may start sequences */

void hw_adc_init(void)
{
    uint32_t i;
    RCC->APB2PCENR |= RCC_ADC1EN;
    ADC1->CTLR1 = ADC_SCAN;                              /* independent; scan: the injected group converts in full */
    ADC1->SAMPTR1 = (7u << 18) | (7u << 21);             /* SMP16, SMP17 = 111: 239.5 cycles */
    ADC1->SAMPTR2 = (6u << 21) | (6u << 15) | 6u;        /* SMP7, SMP5, SMP0 = 110: 71.5 cycles */
    ADC1->RSQR1 = 0;                                     /* L = 0: one regular conversion */
    ADC1->RSQR3 = 7u;                                    /* SQ1 = channel 7 */
    ADC1->ISQR = INJ_VOLT;
    ADC1->CTLR2 = ADC_ADON | ADC_TSVREFE | ADC_JEXTTRIG | ADC_JEXTSEL;   /* power up; JEXTSEL 111: JSWSTART */
    for (i = 0; i < 1000u; i++) {                        /* tSTAB 1 us (and >= 2 ADC clocks before calibration) */
        __asm volatile("nop");
    }
    ADC1->CTLR2 |= ADC_RSTCAL;
    for (i = 0; i < 100000u && (ADC1->CTLR2 & ADC_RSTCAL); i++) {
    }
    ADC1->CTLR2 |= ADC_CAL;
    for (i = 0; i < 100000u && (ADC1->CTLR2 & ADC_CAL); i++) {
    }
    ADC1->CTLR2 |= ADC_CONT;
    ADC1->CTLR2 |= ADC_ADON;                             /* ADON again: start; CONT: keep converting */
    adc_ready = 1;
}

void hw_adc_ms(void)
{
    if (!adc_ready) {
        return;
    }
    if (inj_run != 0u && (ADC1->STATR & ADC_JEOC)) {
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
    }
    ADC1->STATR = ~(uint32_t)ADC_JEOC;                   /* rc_w0: only JEOC cleared */
    if (temp_req) {
        temp_req = 0;
        ADC1->ISQR = INJ_TEMP;
        inj_run = 2;
    } else {
        ADC1->ISQR = INJ_VOLT;
        inj_run = 1;
    }
    ADC1->CTLR2 |= ADC_JSWSTART;
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
}

uint32_t din_ring_pos(void)
{
    return (DIN_RING_WORDS - DMA1_Channel2->CNTR) & (DIN_RING_WORDS - 1u);
}

/* ---------------------------------------------------------------------------------------------------- misc */
static uint32_t reset_flags;

uint32_t hw_reset_flags(void)
{
    return reset_flags;
}

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

/* the stack (Link_sections.ld: _susrstack .. _eusrstack, 2 KB at the top of RAM): painted below the caller's frame at
 * boot; the lowest word no longer holding the pattern is the deepest the stack (main loop + nested ISRs) went */
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

/* independent watchdog: LSI 25-60 kHz / 32, reload 500 -> resets after 0.27-0.64 s (typ. 0.41 s) without a
 * kick. The main loop kicks every pass; its longest stall is a settings write with a page erase (~18 ms). */
void watchdog_kick(void)
{
    IWDG->CTLR = 0xAAAAu;
}

static void watchdog_start(void)
{
    IWDG->CTLR = 0x5555u;                                /* unlock PSCR / RLDR */
    IWDG->PSCR = 3u;                                     /* LSI / 32 */
    IWDG->RLDR = 500u;
    IWDG->CTLR = 0xAAAAu;
    IWDG->CTLR = 0xCCCCu;                                /* start (also starts the LSI) */
}

/* ---------------------------------------------------------------------------------------------------- UART */
/* USART2 115200 8N1. RX: DMA1 ch6 circular into uart_rx. TX: DMA1 ch7 from one of two buffers: uart_write
 * fills the idle buffer, uart_kick sends it. Blocks only when 512 bytes are pending while one is in flight. */
#define UART_RX_SIZE 64u
#define UART_TX_SIZE 512u
static uint8_t uart_rx[UART_RX_SIZE];
static uint8_t uart_tx[2][UART_TX_SIZE];
static uint32_t uart_rx_pos, uart_tx_len;
static uint8_t uart_tx_fill;

int uart_getc(void)
{
    uint32_t w = (UART_RX_SIZE - DMA1_Channel6->CNTR) % UART_RX_SIZE;
    int c;
    if (uart_rx_pos == w) {
        return -1;
    }
    c = uart_rx[uart_rx_pos];
    uart_rx_pos = (uart_rx_pos + 1u) % UART_RX_SIZE;
    return c;
}

static int uart_tx_busy(void)
{
    return (DMA1_Channel7->CFGR & DMA_CFGR1_EN) && DMA1_Channel7->CNTR != 0u;
}

void uart_kick(void)
{
    if (uart_tx_len == 0u || uart_tx_busy()) {
        return;
    }
    DMA1_Channel7->CFGR = 0;
    DMA1_Channel7->MADDR = (uint32_t)uart_tx[uart_tx_fill];
    DMA1_Channel7->CNTR = uart_tx_len;
    DMA1_Channel7->CFGR = DMA_CFGR1_DIR | DMA_CFGR1_MINC | DMA_CFGR1_EN;
    uart_tx_fill ^= 1u;
    uart_tx_len = 0;
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

void hw_init(void)
{
    reset_flags = RCC->RSTSCKR;
    RCC->RSTSCKR |= RCC_RMVF;

    RCC->AHBPCENR |= RCC_DMA1EN;
    RCC->APB2PCENR |= RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_TIM1EN | RCC_SPI1EN;
    RCC->APB1PCENR |= RCC_TIM2EN | RCC_TIM3EN | (1u << 2) /* TIM4 */ | RCC_USART2EN;

    /* TIM2 partial remap 2: CH3 / CH4 on PB10 / PB11 (CH1 / ETR / CH2 stay on PA0 / PA1, unused by TIM2).
     * Others default: TIM1 CH1 / CH2 = PA8 / PA9, TIM3 CH1 / CH3 / CH4 = PA6 / PB0 / PB1, SPI1 SCK / MISO =
     * PA5 / PA6, USART2 = PA2 / PA3 */
    AFIO->PCFR1 = (AFIO->PCFR1 & ~AFIO_PCFR1_TIM2_REMAP) | AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP2;
    pin_cfg(GPIOA, 0, CFG_ANALOG);                       /* the spare input on TP8 (receiver: V_B) */
    pin_cfg(GPIOA, 6, CFG_IN_FLOAT);
    pin_cfg(GPIOA, 5, CFG_AF_PP);                        /* SPI1_SCK, internal only (pad not connected) */
    GPIOA->BSHR = 1u << (FWD_PIN + 16u);                 /* gate closed */
    pin_cfg(GPIOA, FWD_PIN, CFG_OUT_PP);
    pin_cfg(GPIOA, 2, CFG_AF_PP);
    pin_cfg(GPIOA, 3, CFG_IN_FLOAT);
    pin_cfg(GPIOA, 4, CFG_ANALOG);                       /* the supply / bus voltage (R36 / R37 / C29) */
    pin_cfg(GPIOA, 7, CFG_ANALOG);                       /* the current sense */
    pin_cfg(GPIOA, 8, CFG_AF_PP);
    pin_cfg(GPIOA, 9, CFG_AF_PP);
    GPIOA->BCR = 1u << 10;                               /* ODR 0: pull-down. PA10, PB14: unused here */
    pin_cfg(GPIOA, 10, CFG_IN_PULL);
    GPIOB->BCR = 1u << 14;
    pin_cfg(GPIOB, 14, CFG_IN_PULL);
#if BOARD_TESTJIG
    GPIOB->BSHR = 1u << 10;                              /* ODR 1: pull-up. PB10 (TP9): the test jig's pad */
#else
    GPIOB->BCR = 1u << 10;
#endif
    pin_cfg(GPIOB, 10, CFG_IN_PULL);                     /* input (TIM2 CC3E stays 0: no output) */
    led_set(0);
    pin_cfg(GPIOB, 15, CFG_OUT_PP);                      /* status LED D9 */
    pin_cfg(GPIOB, 11, CFG_AF_PP);
    pin_cfg(GPIOB, 0, CFG_AF_PP);
    pin_cfg(GPIOB, 1, CFG_AF_PP);

    /* interrupt priorities (nesting on, bit 7 = preemption): the gate ISR has a 100 us deadline (the first
     * toggle itself is written by DMA), so it may preempt the 20 kHz poll tick (longest path: ring copy) */
    NVIC_SetPriority(TIM4_IRQn, 0x00);
    NVIC_SetPriority(TIM2_IRQn, 0x40);                   /* phase slew (adaptive stagger), once per PWM period */
    NVIC_SetPriority(TIM3_IRQn, 0x40);
    NVIC_SetPriority(SysTicK_IRQn, 0x80);
    NVIC_EnableIRQ(TIM2_IRQn);                           /* only with UIE set by pwm_set_phase */
    NVIC_EnableIRQ(TIM3_IRQn);

    /* DIN edge counter: TIM3 CH1 capture (falling edges of PA6, filter fCK / N = 2) -> TRGO pulse per capture
     * -> TIM4 external clock mode 1 on ITR2. TIM3 itself is started with the PWM (pwm_start). */
    TIM3->CHCTLR1 = (uint16_t)(1u /* CC1S = TI1 */ | (1u << 4) /* IC1F: fCK, N = 2 */);
    TIM3->CCER = TIM_CC1E | TIM_CC1P;
    TIM3->CTLR2 = (uint16_t)(3u << 4);                   /* MMS = 011: compare pulse, also on capture */
    TIM4->PSC = 0;
    TIM4->ATRLR = 0xFFFFu;
    TIM4->SMCFGR = (uint16_t)((2u << 4) /* TS = ITR2 (TIM3) */ | 7u /* SMS: external clock mode 1 */);
    TIM4->CHCTLR1 = (uint16_t)(OCM_FROZEN << 12);         /* CC2: compare only, no pin */
    TIM4->DMAINTENR = 0;
    DMA1_Channel4->PADDR = (uint32_t)&GPIOA->BSHR;       /* TIM4_CH2 = DMA1 channel 4 */
    TIM4->CTLR1 = TIM_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);

    /* DIN sampler: SPI1 master, receive only, 16-bit frames, PCLK2 72 MHz / 2 = 36 MHz,
     * DMA1 ch2 circular. BENCH: confirm the RX-only clock runs continuously (STM32F1 behaviour). A full-duplex
     * fallback would drive MOSI = PA7, the current-sense input, so it would need MOSI unmapped. */
    SPI1->CTLR1 = SPI_CTLR1_MSTR | SPI_CTLR1_SSM | SPI_CTLR1_SSI | SPI_CTLR1_RXONLY | SPI_CTLR1_DFF;
    SPI1->CTLR2 = SPI_CTLR2_RXDMAEN;
    DMA1_Channel2->PADDR = (uint32_t)&SPI1->DATAR;
    DMA1_Channel2->MADDR = (uint32_t)din_ring;
    DMA1_Channel2->CNTR = DIN_RING_WORDS;
    DMA1_Channel2->CFGR = DMA_CFGR1_CIRC | DMA_CFGR1_MINC | DMA_CFGR1_PSIZE_0 | DMA_CFGR1_MSIZE_0 |
                          DMA_CFGR1_PL_0 | DMA_CFGR1_PL_1 | DMA_CFGR1_EN;
    SPI1->CTLR1 |= SPI_CTLR1_SPE;

    /* 20 kHz poll tick */
    SysTick->CMP = 144000000u / POLL_HZ - 1u;            /* HCLK, auto-reload at CMP */
    SysTick->SR = 0;
    SysTick->CTLR = (1u << 5) /* INIT */ | (1u << 3) /* STRE */ | (1u << 2) /* HCLK */ | (1u << 1) /* STIE */ | 1u;
    NVIC_EnableIRQ(SysTicK_IRQn);

    /* debug UART 115200 8N1, DMA both ways (USART2_RX = DMA1 ch6, USART2_TX = DMA1 ch7) */
    USART2->BRR = (uint16_t)(144000000u / 115200u);
    USART2->CTLR3 = USART_CTLR3_DMAR | (1u << 7) /* DMAT */;
    USART2->CTLR1 = USART_CTLR1_TE | USART_CTLR1_RE | USART_CTLR1_UE;
    DMA1_Channel6->PADDR = (uint32_t)&USART2->DATAR;
    DMA1_Channel6->MADDR = (uint32_t)uart_rx;
    DMA1_Channel6->CNTR = UART_RX_SIZE;
    DMA1_Channel6->CFGR = DMA_CFGR1_CIRC | DMA_CFGR1_MINC | DMA_CFGR1_EN;
    DMA1_Channel7->PADDR = (uint32_t)&USART2->DATAR;

    watchdog_start();
}

/* ---------------------------------------------------------------------------------------------------- flash */
#define FLASH_KEY1   0x45670123u
#define FLASH_KEY2   0xCDEF89ABu

static uint32_t store_seq;
static uint8_t store_free;                               /* next slot to write, tracked in RAM (see below) */

/* The chip copies the 64 KB zero-wait flash into SRAM at reset; whether a read after programming returns the new
 * data before the next reset is not documented in the reference manual. So the page is only read here,
 * right after reset, and the write position is tracked in RAM from then on. */
MEASURED int settings_load(settings_t *s, store_ext_t *ext)
{
    uint32_t seq;
    store_free = (uint8_t)store_next_free((const uint8_t *)SETTINGS_ADDR);
    if (store_find_ext((const uint8_t *)SETTINGS_ADDR, s, ext, &seq) < 0) {
        store_ext_clear(ext);
        return -1;
    }
    store_seq = seq;
    return 0;
}

/* Programs `halfwords` half-words at `addr`, erasing the 256-byte page `erase_addr` first (0 = no erase). The
 * flash access clock must stay <= 60 MHz, so HCLK is halved meanwhile (WCH FLASH_Program example); interrupts are
 * off, a phase slew pauses. The gate is closed first: downstream boards just miss a frame or two and hold their
 * last values. Sequences as in WCH's ch32v20x_flash.c; BENCH: verify. Returns the stall estimate in us. */
#define STORE_PROGRAM_US 2000u                           /* BENCH: measure (datasheet: fast page erase 16 ms) */
#define STORE_ERASE_US   16000u
/* the record being programmed: in the main loop's scratch (core/scratch.h), written as its own type and read here as
 * half-words. flash_program's __disable_irq comes first: a compiler barrier, so the record is complete before it is
 * read, inlined or not (LTO) */
typedef char rec_fits_scratch[(sizeof(cal_rec_t) <= CORE_SCRATCH_WORDS * 4u &&
                               sizeof(store_slot_t) <= CORE_SCRATCH_WORDS * 4u &&
                               sizeof(scene_rec_t) <= CORE_SCRATCH_WORDS * 4u) ? 1 : -1];

static uint32_t flash_program(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr)
{
    uint32_t i, stall_us = STORE_PROGRAM_US;
    __disable_irq();                                     /* (the poll tick must not re-arm the gate meanwhile) */
    gate_force_closed();
    /* a phase slew pauses: no period may stay stretched while interrupts are off; it resumes afterwards */
    uint16_t slewing = (uint16_t)((TIM2->DMAINTENR | TIM3->DMAINTENR) & TIM_UIE);
    TIM2->DMAINTENR &= (uint16_t)~TIM_UIE;
    TIM3->DMAINTENR &= (uint16_t)~TIM_UIE;
    if (!cur_spread) {                                   /* spread spectrum: the DMA sets every period's length */
        TIM2->ATRLR = (uint16_t)(cur_steps - 1u);
        TIM3->ATRLR = (uint16_t)(cur_steps - 1u);
    }
    slew_pend2 = slew_pend3 = 0;
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | RCC_HPRE_DIV2;
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    if (erase_addr != 0u) {
        FLASH->MODEKEYR = FLASH_KEY1;                    /* fast mode for the 256-byte page erase */
        FLASH->MODEKEYR = FLASH_KEY2;
        FLASH->CTLR |= FLASH_CTLR_PAGE_ER;
        FLASH->ADDR = erase_addr;
        FLASH->CTLR |= FLASH_CTLR_STRT;
        while (FLASH->STATR & FLASH_STATR_BSY) {
        }
        FLASH->CTLR &= ~FLASH_CTLR_PAGE_ER;
        stall_us += STORE_ERASE_US;
    }
    FLASH->CTLR |= FLASH_CTLR_PG;
    for (i = 0; i < halfwords; i++) {
        *(volatile uint16_t *)(addr + 2u * i) = src[i];
        while (FLASH->STATR & FLASH_STATR_BSY) {
        }
    }
    FLASH->CTLR &= ~FLASH_CTLR_PG;
    FLASH->CTLR |= FLASH_CTLR_LOCK | FLASH_CTLR_FAST_LOCK;

    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | RCC_HPRE_DIV1;
    __enable_irq();
    if (slewing) {
        TIM2->INTFR = (uint16_t)~TIM_UIF;
        TIM3->INTFR = (uint16_t)~TIM_UIF;
        TIM2->DMAINTENR |= TIM_UIE;
        TIM3->DMAINTENR |= TIM_UIE;
    }
    return stall_us;
}

/* The single-page logs (settings, current calibration, energy): the record (in core_scratch) goes into slot *next,
 * the write position its load found at boot and tracked in RAM since; a full page is erased first and the record
 * goes into slot 0 (without may_erase: nothing is written, returns 0). Returns the stall estimate in us. */
static uint32_t page_append(uint32_t page, uint8_t *next, uint32_t bytes, uint32_t n, uint32_t may_erase)
{
    uint32_t idx = *next, erase = 0;
    if (idx >= n) {
        if (!may_erase) {
            return 0;
        }
        erase = page;
        idx = 0;
    }
    *next = (uint8_t)(idx + 1u);                         /* n: the next record erases the page */
    return flash_program(page + idx * bytes, (const uint16_t *)core_scratch, bytes / 2u, erase);
}

/* Appends one 32-byte record (16 half-words, v2 with the extension); erases the page first when all 8 slots are
 * used. */
MEASURED uint32_t settings_store(const settings_t *s, const store_ext_t *ext)
{
    store_pack_ext(s, ext, ++store_seq, (store_slot_t *)core_scratch);
    return page_append(SETTINGS_ADDR, &store_free, STORE_SLOT_BYTES, STORE_SLOTS, 1u);
}

/* Power-on scene log: SCENE_LOG_PAGES pages below the settings page, 16-byte records (core/scene.c). As with the
 * settings, the log is only read right after reset; the write position is tracked in RAM. */
static uint32_t scene_next;                              /* record index to write next */
static uint8_t scene_erase;                              /* erase its page first */
static uint16_t scene_seq;

MEASURED int scene_load(scene_t *sc)
{
    int newest = scene_log_find((const uint8_t *)SCENE_LOG_ADDR, sc, &scene_seq);
    scene_next = (uint32_t)scene_log_next((const uint8_t *)SCENE_LOG_ADDR, newest, &scene_erase);
    return newest < 0 ? -1 : 0;
}

MEASURED uint32_t scene_store(const scene_t *sc)
{
    scene_rec_t *rec = (scene_rec_t *)core_scratch;
    uint32_t idx = scene_next, erase = 0;
    if (scene_erase) {                                   /* entering a page: its oldest records go */
        erase = SCENE_LOG_ADDR + (idx / SCENE_RECS_PAGE) * SCENE_PAGE_BYTES;
    }
    scene_log_pack(sc, ++scene_seq, rec);
    scene_next = (idx + 1u) % SCENE_RECS;
    scene_erase = (uint8_t)(scene_next % SCENE_RECS_PAGE == 0u);
    return flash_program(SCENE_LOG_ADDR + idx * SCENE_REC_BYTES, (const uint16_t *)rec, SCENE_REC_BYTES / 2u, erase);
}

/* Current calibration (core/calstore.h): one page, 64-byte records. As with the settings, the page is only read
 * right after reset; the write position is tracked in RAM. */
static uint32_t cal_seq;
static uint16_t cal_cnt;
static uint8_t cal_free;

MEASURED int cal_load(isense_cal_t *c, uint16_t *count)
{
    cal_free = (uint8_t)cal_next_free((const uint8_t *)CAL_PAGE_ADDR);
    if (cal_find((const uint8_t *)CAL_PAGE_ADDR, c, &cal_seq, &cal_cnt) < 0) {
        cal_cnt = 0;
        *count = 0;
        return -1;
    }
    *count = cal_cnt;
    return c->levels != 0u ? 0 : -1;
}

MEASURED uint32_t cal_store(const isense_cal_t *c, uint16_t *count)
{
    if (c->levels != 0u) {
        cal_cnt++;
    }
    cal_pack(c, ++cal_seq, cal_cnt, (cal_rec_t *)core_scratch);
    *count = cal_cnt;
    return page_append(CAL_PAGE_ADDR, &cal_free, CAL_REC_BYTES, CAL_RECS, 1u);
}

/* Energy log (core/energy.h): one page of 8-byte entries; read only right after reset, the write position is tracked
 * in RAM. */
static uint8_t energy_next;

MEASURED uint32_t energy_load(void)
{
    energy_next = (uint8_t)energy_log_next((const uint8_t *)ENERGY_ADDR);
    return energy_log_find((const uint8_t *)ENERGY_ADDR);
}

MEASURED uint32_t energy_store(uint32_t wh, uint8_t may_erase)
{
    core_scratch[0] = wh;
    core_scratch[1] = ~wh;
    return page_append(ENERGY_ADDR, &energy_next, ENERGY_REC_BYTES, ENERGY_RECS, may_erase);
}

MEASURED uint32_t energy_clear(void)
{
    energy_next = 0;
    return flash_program(ENERGY_ADDR, 0, 0u, ENERGY_ADDR);
}

/* Power-on effect store (core/ultra.h ULTRA_FX_*): read only right after reset (main.c, ultra_fx_load; it keeps the
 * mask and CRCs of what it stored). */
typedef char fx_store_fits[(ULTRA_FX_HDR + ULTRA_LIGHTS * ULTRA_HEAD_BYTES <= 512u && ULTRA_HEAD_BYTES == 98u) ? 1 : -1];

MEASURED uint32_t scene_fx_store(const ultra_t *u, uint8_t mask, const uint8_t crc[ULTRA_LIGHTS])
{
    uint8_t *h = (uint8_t *)core_scratch;
    uint32_t i, us = flash_program(SCENE_FX_ADDR, 0, 0u, SCENE_FX_ADDR);   /* erases page 0: the header goes */
    if (mask == 0u) {
        return us;
    }
    us += flash_program(SCENE_FX_ADDR + 256u, 0, 0u, SCENE_FX_ADDR + 256u);
    core_scratch[2] = 0u;                                /* the 2 spare bytes (10, 11): 0, not leftover scratch */
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        if (mask & (1u << i)) {
            us += flash_program(SCENE_FX_ADDR + ULTRA_FX_HDR + i * ULTRA_HEAD_BYTES, (const uint16_t *)&u->light[i],
                                ULTRA_HEAD_BYTES / 2u, 0u);
        }
        h[5u + i] = crc[i];
    }
    core_scratch[0] = ULTRA_FX_MAGIC;
    h[4] = mask;
    return us + flash_program(SCENE_FX_ADDR, (const uint16_t *)h, ULTRA_FX_HDR / 2u, 0u);   /* the header last */
}

/* Firmware update (core/upd_app.h): one half-word into the app descriptor page's status records (no erase; the
 * page is only erased by the bootloader, together with the descriptor). Returns the stall estimate in us. */
uint32_t hw_flash_half(uint32_t addr, uint16_t v)
{
    return flash_program(addr, &v, 1u, 0u);
}
