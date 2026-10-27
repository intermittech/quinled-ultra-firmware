/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V407 drivers for the Diff-Power-UR-Digital-M: ADC, return channel, console, misc, the SRAM trip. The boot
 * (clock, pins, peripherals) and the flash operation: hw_cold.c; the delay line and the ARGB output: hw_dline.c.
 * Register usage follows the CH32V407 reference manual V1.2 (pages in the comments). Items marked BENCH need silicon. */
#include "ch32v40x.h"
#include "dma_map.h"
#include "hw.h"
#include "hw_priv.h"
#include "../../core/flashrec.h"
#include "../../core/rc.h"
#include "../../core/lto.h"
#include "../../core/perf.h"

uint32_t SystemCoreClock = MCU_HCLK_HZ;

/* ---------------------------------------------------------------------------------------------------- ADC */
/* ADC1 at PCLK2 / 8 = 12.5 MHz (RCC ADCPRE, RM p.25: <= 30 MHz; the RM also names HCLK as the source: /8 keeps
 * <= 25 MHz under either reading, BENCH), no DMA. The temperature sensor needs a 17.1 us sample (DS p.84): 239.5 cycles
 * at 12.5 MHz are 19.2 us, at 25 MHz (the HCLK reading) 9.58 us, and no setting reaches 17.1 us there (ADCPRE /8 and
 * SMP 239.5 are the largest). So hw_adc_init times one 252-cycle conversion against SysTick (HCLK) and keeps the
 * temperature only if its sample is >= 17.1 us; else the temperature (and VDDA, measured with it) stays unavailable.
 * VREFINT (DS: >= 8 us) and the senses meet their samples under both readings. Regular group: IN2 (PA2) the current sense, converted continuously,
 * 71.5 + 12.5 cycles = 6.72 us per result. Injected group, started by the poll tick once per ms (hw_adc_ms): IN1 (bus),
 * IN0 (V_B) twice each at 71.5 cycles (26.9 us), or once a second IN16 / IN17 (temperature, VREFINT) twice each at 239.5
 * cycles (19.2 us each, DS: >= 17.1 us; 80.6 us). Pairs 1 + 3 and 2 + 4 (RM p.181-182: JL = 3 converts JSQ1..4). */
#define INJ_SEQ(a, b) ((3u << 20) | ((uint32_t)(b) << 15) | ((uint32_t)(a) << 10) | ((uint32_t)(b) << 5) | (uint32_t)(a))
#define INJ_VOLT      INJ_SEQ(VSENSE_SUPPLY_CH, 0u)
#define INJ_TEMP      INJ_SEQ(16u, 17u)
#define ADC_TS_MIN_CYC (MCU_HCLK_HZ / 10000000u * 171u) /* 17.1 us in HCLK cycles */

static volatile uint8_t inj_run;
static volatile uint8_t temp_req, temp_done;
static uint8_t adc_ts_ok;                                /* the temperature's sample is long enough (hw_adc_init) */
static volatile uint32_t temp_ts2, temp_vref2;
static volatile uint32_t vs_acc[2], vs_n;
volatile uint8_t adc_ready;
volatile uint16_t hw_diag[HW_DIAG_N];

void hw_adc_init(void)
{
    uint32_t i;
    RCC->PB2PCENR |= RCC_ADC1EN;
    ADC1->CTLR1 = ADC_SCAN;                              /* independent; scan: the injected group converts in full */
    ADC1->SAMPTR1 = (7u << 18) | (7u << 21);             /* SMP16, SMP17 = 111: 239.5 cycles (RM p.178) */
    ADC1->SAMPTR2 = (6u << (3u * ISENSE_ADC_CH)) | (6u << (3u * VSENSE_SUPPLY_CH)) | 6u;   /* SMP2, SMP1, SMP0: 71.5 */
    ADC1->RSQR1 = 0;                                     /* L = 0: one regular conversion */
    ADC1->RSQR3 = ISENSE_ADC_CH;
    ADC1->ISQR = INJ_VOLT;
    ADC1->CTLR2 = ADC_ADON | ADC_JEXTTRIG | ADC_JEXTSEL; /* JEXTSEL 111: JSWSTART (RM p.176) */
    for (i = 0; i < 2000u; i++) {                        /* power-up, and >= 2 ADC clocks before the calibration */
        __asm volatile("nop");
    }
    ADC1->CTLR2 |= ADC_RSTCAL;
    for (i = 0; i < 200000u && (ADC1->CTLR2 & ADC_RSTCAL); i++) {
    }
    ADC1->CTLR2 |= ADC_CAL;
    for (i = 0; i < 200000u && (ADC1->CTLR2 & ADC_CAL); i++) {
    }
    ADC1->CTLR2 |= ADC_TSVREFE;
    {                                                    /* the ADC clock: one IN17 conversion, 239.5 + 12.5 cycles */
        uint32_t t0, d, n = 0;
        const uint32_t s = irq_save();                   /* (a tick inside would lengthen it: 20 us masked, once) */
        ADC1->ISQR = 17u << 15;                          /* JL = 0: JSQ4 only */
        ADC1->STATR = ~(uint32_t)ADC_JEOC;
        t0 = SysTick->CNT;
        ADC1->CTLR2 |= ADC_JSWSTART;
        while (!(ADC1->STATR & ADC_JEOC) && ++n < 100000u) {
        }
        d = SysTick->CNT - t0;
        irq_restore(s);
        if ((int32_t)d < 0) {
            d += SysTick->CMP + 1u;                      /* (one reload: SysTick counts up to CMP) */
        }
        ADC1->STATR = ~(uint32_t)ADC_JEOC;
        ADC1->ISQR = INJ_VOLT;
        adc_ts_ok = (uint8_t)(n < 100000u && d * 479u >= ADC_TS_MIN_CYC * 504u);   /* sample = 239.5 / 252 of it */
    }
    ADC1->CTLR2 |= ADC_CONT;
    ADC1->CTLR2 |= ADC_ADON;                             /* ADON again: start; CONT: keep converting (RM p.176) */
    adc_ready = 1;
}

uint32_t hw_adc_ms(void)
{
    uint32_t sr;
    if (!adc_ready) {
        return 0;
    }
    sr = ADC1->STATR;
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
    }
    ADC1->STATR = ~(uint32_t)ADC_JEOC;                   /* RW0: only JEOC cleared */
    if (temp_req) {
        temp_req = 0;
        ADC1->ISQR = INJ_TEMP;
        inj_run = 2;
    } else {
        ADC1->ISQR = INJ_VOLT;
        inj_run = 1;
    }
    ADC1->CTLR2 |= ADC_JSWSTART;
    return inj_run - 1u;
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
    temp_req = adc_ts_ok;                                /* (a too-short sample: no temperature at all) */
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

/* DMA1's transfer error flags of channels 1..7 (INTFR, RM p.155-156): counted and cleared. Channel 10 (ARGB, flags in
 * EXTEM_INTFR) belongs to the ARGB driver (hw_dline.h argb_hw_status: a failed frame counts as lost, encoder lost). */
void hw_dma_check(void)
{
    uint32_t f = DMA1->INTFR & 0x08888888u, i;
    for (i = 0; i < 7u; i++) {
        if (f & (DMA_TEIF1 << (4u * i))) {
            hw_diag[2u + i]++;
        }
    }
    DMA1->INTFCR = f;
}

/* ---------------------------------------------------------------------------------------------------- misc */
uint32_t hw_uid_hash(void)
{
    const volatile uint32_t *u = (const volatile uint32_t *)0x1FFFF7E8u;   /* ESIG UID (RM p.577) */
    uint32_t h = (u[0] ^ u[1] ^ u[2]) * 0x9E3779B9u;
    return h ^ (h >> 16);
}

void led_set(uint8_t on)
{
    GPIOA->BSHR = on ? (1u << 4) : (1u << (4 + 16));   /* status LED on PA4, active high */
}

uint32_t hw_hsi_trim(void)
{
    return (RCC->CTLR & RCC_HSITRIM) >> RCC_HSITRIM_SHIFT;
}

void hw_hsi_trim_set(uint32_t trim)
{
    RCC->CTLR = (RCC->CTLR & ~RCC_HSITRIM) | ((trim & 31u) << RCC_HSITRIM_SHIFT);
}

/* ------------------------------------------------------------------------------------------- return channel */
/* USART7 TX on PA9 (remap 11), 8N1, at PCLK1 100 MHz, interrupt-driven at priority 0 (VTF slot 1). RX and CK stay
 * off (remap 11 maps them onto DE_OUT / BLANK_N). PA9 drives only from DE high to the burst's end: otherwise it is an
 * input, so a later port-2 relay (its receive output reaches this line too) is not overdriven (the line idles high).
 * Receive read-back is unavailable during a reply (as on the V20x Digital-M). */
static const uint8_t *rc_next;
static uint32_t rc_left;
static uint8_t rc_relaying;                              /* a relay window: DE stays high around this board's burst */
typedef char rc_one_lead_byte[RC_LEAD_BYTES == 1u ? 1 : -1];

static void rc_release(void)
{
    RC_USART->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE;   /* no TXE / TC interrupt: a burst still running is cut */
    if (!rc_relaying) {
        GPIOA->BCR = 1u << RC_DE_PIN;                    /* DE low: the hub's line again */
    }
    pin_cfg(RC_TX_GPIO, RC_TX_PIN, CFG_IN_FLOAT);        /* PA9 released (in a window: the relay node is port 2's again) */
}

/* The CHAIN relay window (main_ur_digital_m.c): on = the turnaround: port 2 listens (DE_OUT low: its receive output drives
 * the relay node, PA9's line), then the line above is sampled with DE still low: LOW = still driven (returns 1: no relay and
 * no answer in this window, the input listens too); else DE high: the relay node drives the line above (the boards behind,
 * and this board's own burst on PA9). off = forward again: the input listens, port 2 drives downstream (the gate is the
 * caller's) */
uint32_t rc_hw_relay(uint32_t on)
{
    if (on) {
        GPIOA->BCR = 1u << 10;                           /* DE_OUT low */
        if ((GPIOB->INDR & (1u << 4)) == 0u) {           /* RO (PB4) LOW: the segment above is still driven */
            return 1;
        }
        rc_relaying = 1;
        GPIOA->BSHR = 1u << RC_DE_PIN;
        return 0;
    }
    rc_relaying = 0;
    rc_release();
    GPIOA->BSHR = 1u << 10;                              /* DE_OUT high */
    return 0;
}

void rc_hw_dir(uint32_t mode)
{
    if (mode == RC_DIR_ANSWER) {
        GPIOA->BCR = 1u << RC_BLANK_PIN;                 /* BLANK_N LOW first: the strip line held LOW */
        pin_cfg(RC_TX_GPIO, RC_TX_PIN, CFG_AF_PP);       /* PA9 drives: the USART idles HIGH */
        GPIOA->BSHR = 1u << RC_DE_PIN;                   /* then DE high: the line HIGH */
    } else {
        rc_release();
    }
}

void rc_hw_answer_local(void)
{
    pin_cfg(RC_TX_GPIO, RC_TX_PIN, CFG_AF_PP);
    GPIOA->BSHR = 1u << RC_DE_PIN;
}

uint32_t rc_hw_send(const uint8_t *b, uint32_t n, uint16_t brr)
{
    uint32_t t = POLL_CNT(), d;
    RC_USART->BRR = brr;
    do {                                                 /* the line HIGH RC_PREAMBLE_CYC (DE high since rc_hw_dir) */
        d = POLL_CNT() - t;
        if ((int32_t)d < 0) {
            d += MCU_HCLK_HZ / POLL_HZ;
        }
    } while (d < RC_PREAMBLE_CYC);
    if (n == 0u) {
        rc_release();
        return 0;
    }
    rc_next = b;
    rc_left = n;
    BARRIER();
    RC_USART->STATR = ~(uint32_t)USART_STATR_TC;         /* TC is RW0 (RM p.276) */
    RC_USART->DATAR = 0xFFu;                             /* RC_LEAD_BYTES */
    RC_USART->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE | USART_CTLR1_TXEIE;
    return 0;
}

uint32_t rc_hw_busy(void)
{
    return (RC_USART->CTLR1 & (USART_CTLR1_TXEIE | USART_CTLR1_TCIE)) != 0u;
}

void rc_uart_isr(void);
void rc_uart_isr(void)
{
    BARRIER();
    if (RC_USART->CTLR1 & USART_CTLR1_TXEIE) {
        if (rc_left != 0u) {
            rc_left--;
            RC_USART->DATAR = *rc_next++;
            return;
        }
        RC_USART->STATR = ~(uint32_t)USART_STATR_TC;
        RC_USART->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE | USART_CTLR1_TCIE;
        return;
    }
    rc_release();                                        /* TC: the last stop bit is out */
    dl_tx_done();
}

/* ---------------------------------------------------------------------------------------------------- stack */
#define STACK_PAINT 0xC5A3965Au
extern uint32_t _susrstack[], _eusrstack[];

void stack_paint(void)
{
    uint32_t *p = _susrstack, *sp;
    __asm volatile("mv %0, sp" : "=r"(sp));
    for (sp -= 8; p < sp; p++) {
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

void watchdog_kick(void)
{
    IWDG->CTLR = 0xAAAAu;
}

/* ---------------------------------------------------------------------------------------------------- console */
/* USART1 115200 8N1 on PA6 (remap 11), TX only (the release console takes no input). A ring of CON_SIZE bytes, filled
 * by the main loop, emptied by the TXE interrupt (priority 0xC0: the tick's preemption group, so neither interrupts the
 * other). During a flash operation the interrupt is masked (hw_cold.c); the shift register finishes its byte. */
#define CON_SIZE 512u
static uint8_t con_buf[CON_SIZE];
static volatile uint32_t con_w, con_r;                   /* free-running: the main loop writes con_w, the ISR con_r */

void con_uart_isr(void);
void con_uart_isr(void)
{
    uint32_t r = con_r;
    if (r != con_w) {
        USART1->DATAR = con_buf[r % CON_SIZE];
        con_r = r + 1u;
    }
    if (con_r == con_w) {
        USART1->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE; /* empty: TXE interrupt off */
    }
}

uint32_t uart_room(void)
{
    return CON_SIZE - (con_w - con_r);
}

uint32_t uart_busy(void)
{
    return con_w != con_r || !(USART1->STATR & USART_STATR_TC);
}

void uart_kick(void)
{
    if (con_w != con_r && !(USART1->CTLR1 & USART_CTLR1_TXEIE)) {
        USART1->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE | USART_CTLR1_TXEIE;
    }
}

void uart_write(const char *s)
{
    while (*s) {
        while (con_w - con_r == CON_SIZE) {              /* full: send what is there and wait */
            uart_kick();
        }
        con_buf[con_w % CON_SIZE] = (uint8_t)*s++;
        BARRIER();
        con_w++;
    }
}

/* ---------------------------------------------------------------------------------------------------- flash bits */
void hw_records_window(uint32_t open)
{
    (void)open;                  /* the flash access clock (HCLK / 2 = 100 MHz) is within its limit for every area */
}

uint32_t hw_vdd_low(void)
{
    return (PWR->CSR & PWR_CSR_PVDO) != 0u;
}

uint32_t hw_boot_open(void)
{
    return (FLASH->WPR & 0xFu) != 0u;                    /* sectors 0-3 (the bootloader) writable (RM p.585) */
}

uint32_t hw_option_bytes(uint16_t ob[8])
{
    const volatile uint16_t *o = (const volatile uint16_t *)HW_OB_ADDR;
    uint32_t i, ok = 1, a;
    for (i = 0; i < 8u; i++) {
        a = o[i];
        ok &= a == o[i] && ((a ^ (a >> 8)) & 0xFFu) == 0xFFu;
        ob[i] = (uint16_t)(a & 0xFFu);
    }
    return ok;
}

/* an exception during a flash operation (isr.S flash_fault, then the watchdog): the strip on the direct path, the
 * replay selected, the hub's line released (as the pulls hold them after the reset) */
RAMCODE void hw_flash_trip(void)
{
    GPIOB->BCR = (1u << 6) | (1u << 7);                  /* MUX_SEL direct, SOURCE_SEL replay */
    GPIOA->BCR = 1u << RC_DE_PIN;                        /* input DE low */
}
