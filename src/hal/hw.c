/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V203F8U6 drivers: clock, GPIO, PWM timers + dither DMA, DIN edge counter / gate, DIN sampler, flash.
 * Register usage follows the WCH CH32FV2x_V3xRM reference manual V2.5. Items marked BENCH need confirmation on real
 * hardware. */
#include "ch32v20x.h"
#include "hw.h"
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

static uint32_t cur_steps;
static uint8_t cur_stagger, cur_len, cur_spread;
static uint16_t spread_arr[PATTERN_MAX_LEN];         /* spread spectrum: period - 1 of each table entry */
volatile uint8_t pwm_forced;                  /* isense trip: all PWM outputs forced low */
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

/* hw_init's pins (board.h), in order: the pin, its port, an ODR write before the mode, and the CFG nibble (bits 8-11) */
#define PIN_B    0x10u                                   /* GPIOB (else GPIOA) */
#define PIN_HI   0x20u                                   /* ODR 1 first (BSHR set) */
#define PIN_LO   0x40u                                   /* ODR 0 first (BSHR reset) */
#define PIN_CLR  0x80u                                   /* ODR 0 first (BCR) */
#define PIN(port, pin, odr, cfg) ((uint16_t)((pin) | (port) | (odr) | ((cfg) << 8)))
static const uint16_t pin_tab[] = {
    PIN(0, 0, 0, CFG_ANALOG),                            /* PA0: the spare input on TP8 (receiver: V_B) */
    PIN(0, 6, 0, CFG_IN_FLOAT),                          /* PA6: DIN */
    PIN(0, 5, 0, CFG_AF_PP),                             /* PA5: SPI1_SCK, internal only (pad not connected) */
    PIN(0, FWD_PIN, PIN_LO, CFG_OUT_PP),                 /* PA1: FWD_EN, the gate closed */
    PIN(0, 2, 0, CFG_AF_PP),                             /* PA2 / PA3: the console */
    PIN(0, 3, 0, CFG_IN_FLOAT),
    PIN(0, 4, 0, CFG_ANALOG),                            /* PA4: the supply / bus voltage (R36 / R37 / C29) */
    PIN(0, 7, 0, CFG_ANALOG),                            /* PA7: the current sense */
    PIN(0, 8, 0, CFG_AF_PP),                             /* PA8 / PA9: PWM G / R */
    PIN(0, 9, 0, CFG_AF_PP),
#if BOARD_RETURN
    PIN(0, 10, PIN_HI, CFG_OUT_PP),                      /* PA10: forward mode, the output transceiver drives downstream */
    PIN(PIN_B, 14, PIN_CLR, CFG_OUT_PP),                 /* PB14: ... the input one listens upstream */
#else
    PIN(0, 10, PIN_CLR, CFG_IN_PULL),                    /* PA10, PB14: pull-down, unused here */
    PIN(PIN_B, 14, PIN_CLR, CFG_IN_PULL),
#endif
#if BOARD_TESTJIG
    PIN(PIN_B, 10, PIN_HI, CFG_IN_PULL),                 /* PB10 (TP9): pull-up, the test jig's pad (TIM2 CC3E stays 0) */
#else
    PIN(PIN_B, 10, PIN_CLR, CFG_IN_PULL),
#endif
    PIN(PIN_B, 15, PIN_LO, CFG_OUT_PP),                  /* PB15: status LED D9, off */
    PIN(PIN_B, 11, 0, CFG_AF_PP),                        /* PB11: PWM B */
    PIN(PIN_B, 0, 0, CFG_AF_PP),                         /* PB0 / PB1: PWM W2 / W1 */
    PIN(PIN_B, 1, 0, CFG_AF_PP),
};

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
static volatile uint8_t adc_ready;                       /* hw_adc_init done: the poll tick may start sequences (0
                                                            during flash_op too: the SRAM tick does not run them) */
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

#if BOARD_RETURN
/* ------------------------------------------------------------------------------------------- return channel */
/* USART3 TX only (RE 0: its RX pin PB11 is PWM_B), 8N1, by interrupt: every DMA channel is taken (USART3_TX's, ch2,
 * is the DIN sampler's). Its entry: VTF slot 1 (0.6.5's unified trap_entry dispatches only SysTick and TIM2-4;
 * slot 0 is flash_op's), priority 0x00, so it preempts the poll tick (a byte every 10 us at 1 Mbaud; a late one only
 * stretches the stop bit). PB10 is driven only during a burst; otherwise an input with its weak pull-down, and the relay node
 * follows the output transceiver's RO (1 k against the 100 R from PB10). BENCH: the node's levels, DIN's read-back. */
static const uint8_t *rc_next;
static uint32_t rc_left;
extern void rc_uart_entry(void);                         /* isr.S */

static void rc_release(void)
{
    USART3->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE;     /* no TXE / TC interrupt */
    pin_cfg(GPIOB, 10, CFG_IN_PULL);
}

static void rc_hw_init(void)
{
    RCC->APB1PCENR |= 1u << 18;                          /* USART3EN */
    USART3->BRR = 144u;                                  /* 1 Mbaud at PCLK1 144 MHz (rc_brr per burst) */
    rc_release();
    NVIC_SetPriority(USART3_IRQn, 0x00);
    PFIC->VTFIDR[1] = USART3_IRQn;
    PFIC->VTFADDR[1] = (uint32_t)rc_uart_entry | 1u;
    NVIC_EnableIRQ(USART3_IRQn);
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
    rc_next = b + 1;
    rc_left = n - 1u;
    USART3->STATR = (uint16_t)~USART_STATR_TC;
    USART3->DATAR = b[0];
    USART3->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE | USART_CTLR1_TXEIE;
    return 0;
}

void rc_uart_isr(void);                                  /* USART3, entered via isr.S (VTF, HPE + mret) */
void rc_uart_isr(void)
{
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
static uint8_t uart_rx_drop;                             /* flash_op dropped bytes: 1 = tell main.c, 2 = to CR / LF */

static uint32_t uart_rx_w(void)
{
    return (UART_RX_SIZE - DMA1_Channel6->CNTR) % UART_RX_SIZE;
}

int uart_getc(void)
{
    uint32_t w = uart_rx_w();
    int c;
    if (uart_rx_drop == 1u) {
        uart_rx_drop = 2;
        return -2;                                       /* (main.c: the line being typed is cancelled) */
    }
    if (uart_rx_pos == w) {
        return -1;
    }
    c = uart_rx[uart_rx_pos];
    uart_rx_pos = (uart_rx_pos + 1u) % UART_RX_SIZE;
    if (uart_rx_drop) {
        if (c == '\r' || c == '\n') {
            uart_rx_drop = 0;                            /* the rest of the cut line is gone, the next one counts */
        }
        return -1;
    }
    return c;
}

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

void hw_init(void)
{
    reset_flags = RCC->RSTSCKR;
    RCC->RSTSCKR |= RCC_RMVF;

    RCC->AHBPCENR |= RCC_DMA1EN;
    RCC->APB2PCENR |= RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_TIM1EN | RCC_SPI1EN;
    RCC->APB1PCENR |= RCC_TIM2EN | RCC_TIM3EN | (1u << 2) /* TIM4 */ | RCC_USART2EN | RCC_PWREN;
    /* (0.6.5) the PVD at 2.9 V (as the bootloader): flash_op starts nothing while VDD is below it (DS table 4-17:
     * programming needs >= 2.4 V; a dip write may run while the supply goes away) */
    PWR->CTLR = (PWR->CTLR & ~PWR_CTLR_PLS) | PWR_CTLR_PLS_2V9 | PWR_CTLR_PVDE;

    /* TIM2 partial remap 2: CH3 / CH4 on PB10 / PB11 (CH1 / ETR / CH2 stay on PA0 / PA1, unused by TIM2).
     * Others default: TIM1 CH1 / CH2 = PA8 / PA9, TIM3 CH1 / CH3 / CH4 = PA6 / PB0 / PB1, SPI1 SCK / MISO =
     * PA5 / PA6, USART2 = PA2 / PA3 */
    AFIO->PCFR1 = (AFIO->PCFR1 & ~AFIO_PCFR1_TIM2_REMAP) | AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP2;
    /* the pins (board.h), in this order, each with its ODR write first where one is needed (PIN_HI / PIN_LO before a
     * pull input or an output: the level before the mode). One table (0.6.6, review round 5: the 20 pin_cfg calls
     * inlined were ~210 B more). PIN_LO writes BSHR's reset half, PIN_CLR the BCR, as the 0.6.5 code did. */
    {
        uint32_t i;
        for (i = 0; i < sizeof pin_tab / sizeof pin_tab[0]; i++) {
            uint32_t t = pin_tab[i], pin = t & 15u;
            GPIO_TypeDef *g = (t & PIN_B) ? GPIOB : GPIOA;
            if (t & PIN_HI) {
                g->BSHR = 1u << pin;
            } else if (t & PIN_LO) {
                g->BSHR = 1u << (pin + 16u);
            } else if (t & PIN_CLR) {
                g->BCR = 1u << pin;
            }
            pin_cfg(g, pin, t >> 8);
        }
    }

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
#if BOARD_RETURN
    rc_hw_init();
#endif

    watchdog_start();
}

/* ---------------------------------------------------------------------------------------------------- flash */
/* flash_op (core/flashrec.h; the stores: core/flashrec.c; when main.c writes: flash_may, Quindor's policy). 0.6.4
 * (review round 4: gpt-hw H4 / H8 / H9; the design: gpt-flashsafe):
 *  - HCLK / 2 during an erase or program (RM 32.1 note 2, WCH's FLASH example; BENCH / WCH: the datasheet's 60 MHz
 *    note): the PWM timers count at half rate meanwhile (the duty kept, each period twice as long). PCLK2 stays at
 *    72 MHz (PPRE2 /1 instead of /2): the ADC (12 MHz, PA7 keeps converting) and the DIN sampler keep their clocks.
 *    SysTick's compare is rescaled (50 us ticks), its phase kept.
 *  - The current trip keeps running (0.6.3: interrupts off for the whole operation): interrupts stay on, but only
 *    SysTick's, through VTF slot 0 to flash_tick in SRAM (main.c flash_poll_isr: the time base and the fast trip);
 *    every other interrupt is off meanwhile and mtvec points to an SRAM handler (an exception: the outputs off, then
 *    the watchdog). The SRAM code (.ramtext) is copied by the startup with .data. The erase / program commands stay
 *    in the zero-wait flash (RM 32.2.1); the BSY wait runs from SRAM. BENCH / WCH: that split; the trip serviced
 *    throughout BSY.
 *  - Status (RM 32.4.3, 32.5): EOP / WRPRTERR cleared before and checked after each operation; the result is FLASH_OK
 *    or FLASH_ERR_*, no longer a duration (the time bases run on meanwhile). A BSY that never ends (~0.1 s): the board
 *    holds the slow clock with the trip running until the watchdog resets it (144 MHz with the flash busy: unsafe).
 *  - USART2: a byte on the line is finished first (TC); main.c starts a regular write only while nothing is queued
 *    (uart_busy). RX keeps running: bytes received meanwhile (at half the baud: garbage) are dropped, and with them the
 *    line being typed (uart_getc returns -2 once; the rest of that line, up to CR / LF, is dropped too).
 *  - The gate is closed (the tick does no frame work meanwhile): downstream boards miss a frame or two and hold their
 *    last values. A phase slew pauses (no period stays stretched), the injected ADC sequences too. */
#define FLASH_KEY1      0x45670123u
#define FLASH_KEY2      0xCDEF89ABu
#define FLASH_BSY_POLLS 2000000u                         /* ~0.1-0.2 s of BSY polls at 72 MHz: the flash is stuck */
#define FLASH_TC_POLLS  4000u                            /* > one byte at 115200 baud (87 us) */
#ifndef FLASH_VERIFY
#define FLASH_VERIFY 0                                   /* 1: read each half-word back (BENCH: a read after programming
                                                            may come from the copy made at reset) */
#endif
extern void flash_tick(void), flash_fault(void);         /* isr.S, .ramtext */

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

/* the SRAM path's trip: all five outputs forced low (as pwm_force_off(1)); never back to PWM */
RAMCODE void hw_flash_trip(void)
{
    PWM_TRIP();
}

/* the flash's status once it is done (SRAM: nothing is fetched from the flash meanwhile) */
static RAMCODE uint32_t flash_wait(void)
{
    uint32_t n = 0, sr;
    while ((sr = FLASH->STATR) & FLASH_STATR_BSY) {
        if (++n == FLASH_BSY_POLLS) {
            for (;;) {
                /* stuck: no watchdog kick; the tick (SRAM) keeps the trip running until the reset */
            }
        }
    }
    return (sr & FLASH_STATR_WRPRTERR) ? FLASH_ERR_WRP : (sr & FLASH_STATR_EOP) ? FLASH_OK : FLASH_ERR_EOP;
}

/* the clocks of a flash operation (slow: HCLK / 2, PCLK2 72 MHz) or back: SysTick (HCLK) rescaled with its phase, a
 * tick that came due meanwhile taken now (interrupts off) */
static void flash_clock(uint32_t cfgr, uint32_t slow)
{
    uint32_t ctl = SysTick->CTLR & ~(1u << 5), cnt;      /* (not INIT) */
    SysTick->CTLR = ctl & ~1u;
    cnt = (uint32_t)SysTick->CNT;
    if (SysTick->SR & 1u) {
        flash_poll_isr();
    }
    NVIC_ClearPendingIRQ(SysTicK_IRQn);
    RCC->CFGR0 = cfgr;
    SysTick->CMP = (slow ? 72000000u : 144000000u) / POLL_HZ - 1u;
    SysTick->CNT = slow ? cnt / 2u : cnt * 2u;
    SysTick->CTLR = ctl;
}

/* the slew and gate interrupts (hw_init enables them with SysTick; nothing else is enabled, and no VTF slot is used;
 * the receiver: also its return channel's USART3, on VTF slot 1) */
#if BOARD_RETURN
#define FLASH_IRQ1 ((1u << (TIM2_IRQn - 32)) | (1u << (TIM3_IRQn - 32)) | (1u << (TIM4_IRQn - 32)) | \
                    (1u << (USART3_IRQn - 32)))
#else
#define FLASH_IRQ1 ((1u << (TIM2_IRQn - 32)) | (1u << (TIM3_IRQn - 32)) | (1u << (TIM4_IRQn - 32)))
#endif

uint32_t flash_op(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr)
{
    uint32_t i, r = FLASH_OK, cfgr, mtvec, rxw, adc = adc_ready;
    uint16_t slewing;
    BARRIER();                                           /* the record (written as its own type) is complete */
    if (PWR->CSR & PWR_CSR_PVDO) {
        return FLASH_ERR_VDD;                            /* (0.6.5) VDD below 2.9 V: the write stays pending */
    }
    adc_ready = 0;                                       /* no new injected sequence (hw_adc_ms) */
    for (i = 0; i < FLASH_TC_POLLS && !(USART2->STATR & USART_STATR_TC); i++) {
    }
    __disable_irq();
    gate_force_closed();
    slewing = (uint16_t)((TIM2->DMAINTENR | TIM3->DMAINTENR) & TIM_UIE);
    TIM2->DMAINTENR &= (uint16_t)~TIM_UIE;
    TIM3->DMAINTENR &= (uint16_t)~TIM_UIE;
    if (!cur_spread) {                                   /* spread spectrum: the DMA sets every period's length */
        TIM2->ATRLR = (uint16_t)(cur_steps - 1u);
        TIM3->ATRLR = (uint16_t)(cur_steps - 1u);
    }
    slew_pend2 = slew_pend3 = 0;
    rxw = uart_rx_w();
    /* only SysTick, through VTF slot 0 into SRAM; mtvec to the SRAM fault handler */
    cfgr = RCC->CFGR0;
    __asm volatile("csrr %0, mtvec" : "=r"(mtvec));
    PFIC->IRER[1] = FLASH_IRQ1;
    PFIC->VTFIDR[0] = SysTicK_IRQn;
    PFIC->VTFADDR[0] = (uint32_t)flash_tick | 1u;
    __asm volatile("csrw mtvec, %0" ::"r"((uint32_t)flash_fault) : "memory");
    flash_clock((cfgr & ~(RCC_HPRE | RCC_PPRE2)) | RCC_HPRE_DIV2, 1);
    __enable_irq();
    watchdog_kick();
    __disable_irq();                                     /* (each key pair back to back) */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    FLASH->MODEKEYR = FLASH_KEY1;                        /* fast mode: the 256-byte page erase */
    FLASH->MODEKEYR = FLASH_KEY2;
    __enable_irq();
    if (FLASH->CTLR & (FLASH_CTLR_LOCK | FLASH_CTLR_FAST_LOCK)) {
        r = FLASH_ERR_KEY;
    } else {
        if (erase_addr != 0u) {
            FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;   /* (write 1 to clear) */
            FLASH->CTLR |= FLASH_CTLR_PAGE_ER;
            FLASH->ADDR = erase_addr;
            FLASH->CTLR |= FLASH_CTLR_STRT;
            r = flash_wait();
            FLASH->CTLR &= ~(FLASH_CTLR_PAGE_ER | FLASH_CTLR_STRT);
            watchdog_kick();
        }
        if (r == FLASH_OK && halfwords != 0u) {
            FLASH->CTLR |= FLASH_CTLR_PG;
            for (i = 0; i < halfwords && r == FLASH_OK; i++) {
                FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;
                *(volatile uint16_t *)(addr + 2u * i) = src[i];
                r = flash_wait();
#if FLASH_VERIFY
                if (r == FLASH_OK && *(volatile const uint16_t *)(addr + 2u * i) != src[i]) {
                    r = FLASH_ERR_VERIFY;
                }
#endif
            }
            FLASH->CTLR &= ~FLASH_CTLR_PG;
            watchdog_kick();
        }
    }
    FLASH->CTLR |= FLASH_CTLR_LOCK | FLASH_CTLR_FAST_LOCK;
    __disable_irq();
    flash_clock(cfgr, 0);
    PFIC->VTFADDR[0] = 0;
    __asm volatile("csrw mtvec, %0" ::"r"(mtvec) : "memory");
    PFIC->IENR[1] = FLASH_IRQ1;
    if (uart_rx_w() != rxw || (USART2->STATR & (USART_STATR_ORE | USART_STATR_FE | USART_STATR_NE))) {
        (void)USART2->DATAR;                             /* (STATR then DATAR: the error flags clear) */
        uart_rx_pos = uart_rx_w();                       /* received at the halved baud: dropped, the line too */
        uart_rx_drop = 1;
    }
    adc_ready = (uint8_t)adc;
    __enable_irq();
    if (slewing) {
        TIM2->INTFR = (uint16_t)~TIM_UIF;
        TIM3->INTFR = (uint16_t)~TIM_UIF;
        TIM2->DMAINTENR |= TIM_UIE;
        TIM3->DMAINTENR |= TIM_UIE;
    }
    return r;
}
