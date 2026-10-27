/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V407 bootloader backend (BOARD 3, the Diff-Power-UR-Digital-M): no interrupts. DIN sampling at 50 MHz (SPI1
 * RX-only, PCLK2 100 MHz / 2 with HSRXEN), flash erase / program and their waits in the zero-wait flash (RM p.579:
 * only there) with monotonic deadlines, the strip held LOW and the
 * mux direct throughout. Register usage: the CH32V407 reference manual V1.2 (pages in the comments); register
 * sequences and declared timing still require physical qualification. */
#include "ch32v40x.h"
#include "../../../bootloader/bl_hw.h"
#include "../../../bootloader/bl_core.h"
#include "boot_api_layout.h"
#include "upd_forward.h"

uint16_t hw_ring[HW_RING_WORDS] __attribute__((aligned(4), section(".noinit")));
volatile uint8_t hw_flash_stall;
static uint32_t hb_at_reset;
static uint32_t clock_khz = MCU_RESET_HSI_HZ / 1000u, deadline_us, slow_start, slow_budget;
static uint8_t options_trusted;

/* the compiler may call these for struct copies / clearing loops (no C library is linked) */
__attribute__((used, externally_visible)) void *memcpy(void *d, const void *s, unsigned int n)
{
    uint8_t *a = (uint8_t *)d;
    const uint8_t *b = (const uint8_t *)s;
    while (n--) {
        *a++ = *b++;
    }
    return d;
}

__attribute__((used, externally_visible)) void *memset(void *d, int c, unsigned int n)
{
    uint8_t *a = (uint8_t *)d;
    while (n--) {
        *a++ = (uint8_t)c;
    }
    return d;
}

#define CFG_IN_FLOAT 0x4u
#define CFG_OUT_PP   0x3u
#define CFG_OUT_2M   0x2u
#define CFG_AF_PP    0xBu

static void pin_cfg(GPIO_TypeDef *g, uint32_t pin, uint32_t cfg)
{
    volatile uint32_t *r = pin < 8u ? &g->CFGLR : &g->CFGHR;
    uint32_t sh = (pin & 7u) * 4u;
    *r = (*r & ~(0xFu << sh)) | (cfg << sh);
}

/* ------------------------------------------------------------------------------------------------ reset / boot */
/* BLANK_N (PA8) LOW before a boot path is chosen; the app keeps it until its receive path is ready */
void hw_blank_early(void)
{
    RCC->PB2PCENR |= RCC_IOPAEN;
    GPIOA->BCR = 1u << 8;
    pin_cfg(GPIOA, 8, CFG_OUT_PP);
}

uint32_t hw_reset_flags_take(void)
{
    uint32_t f = RCC->RSTSCKR;
    RCC->RSTSCKR |= RCC_RMVF;
    return f;
}

void hw_crc_on(void)
{
    hb_at_reset = RCC->HBPCENR;
    RCC->HBPCENR |= RCC_CRCEN;
}

void hw_crc_off(void)
{
    RCC->HBPCENR = hb_at_reset;
}

uint32_t bl_plat_crc32(uint32_t addr, uint32_t nwords)
{
    const volatile uint32_t *p = (const volatile uint32_t *)addr;
    CRC->CTLR = 1u;                           /* reset to 0xFFFFFFFF (RM p.45) */
    while (nwords--) {
        CRC->DATAR = *p++;
    }
    return CRC->DATAR;
}

void hw_soft_reset(void)
{
    NVIC_SystemReset();
    for (;;) {
    }
}

/* The descriptor's entry is physical (inside the body); the app is linked at the zero alias, so the jump uses the
 * alias (docs/update.md: a target may translate to its reset alias when jumping, never when checking bounds). */
void hw_jump_app(void)
{
    uint32_t entry = upd_get32(bl_plat_ptr(UPD_APP_DESC) + UPD_D_ENTRY);
    SysTick->CTLR = 0;
    SysTick->CNT = 0;
    hw_crc_off();
    ((void (*)(void))(entry - MCU_BOOT_API_FLASH_BASE + MCU_BOOT_API_LINK_BASE))();
    for (;;) {
    }
}

/* ------------------------------------------------------------------------------------------------ clock, time */
static uint32_t ms_count, tick_last, tick_rem;

uint32_t hw_ticks(void)
{
    return SysTick->CNT;                      /* 32-bit (RM p.80) */
}

uint32_t hw_ms(void)
{
    uint32_t now = hw_ticks(), d = now - tick_last + tick_rem;
    tick_last = now;
    ms_count += d / clock_khz;
    tick_rem = d % clock_khz;
    return ms_count;
}

uint32_t bl_plat_ms(void) { return hw_ms(); }

uint32_t bl_plat_ticks(void) { return hw_ticks(); }

const uint8_t *bl_plat_uid(void)
{
    return (const uint8_t *)0x1FFFF7E8u;      /* ESIG UID, 12 bytes (RM p.577) */
}

static uint32_t physical_us(void)
{
    uint32_t ms = hw_ms();
    return ms * 1000u + tick_rem * 1000u / clock_khz;
}

void bl_plat_begin(uint32_t deadline_ms)
{
    (void)bl_plat_boot_ticks();
    deadline_us = physical_us() + deadline_ms * 1000u - 100u;   /* 100 us for failure / reset cleanup */
    hw_kick();
}

/* the app's clock (hw_cold.c SystemInit, RM p.17-38): HSI trimmed low (decided 2026-10-22), PLL x10 = 200 MHz,
 * SYSCLK = HCLK, PCLK1 = PCLK2 = 100 MHz; the time base counts the trimmed clock's nominal rate */
static void clock_200(void)
{
    RCC->CTLR = (RCC->CTLR & ~RCC_HSITRIM) | RCC_HSION |
                ((MCU_HSI_TRIM_RESET - MCU_HSI_BOOT_TRIM_STEPS) << RCC_HSITRIM_SHIFT);
    while ((RCC->CTLR & RCC_HSIRDY) == 0u) {
    }
    RCC->CFGR0 = (RCC->CFGR0 & ~(RCC_SW | RCC_HPRE | RCC_PPRE1 | RCC_PPRE2 | RCC_ADCPRE | RCC_PLLSRC | RCC_PLLMUL |
                                 RCC_ADCSRC)) | RCC_PPRE1_DIV2 | RCC_PPRE2_DIV2 | RCC_ADCPRE_DIV8 | RCC_PLLMUL10;
    RCC->CFGR2 = (RCC->CFGR2 & ~(RCC_SYSPLLSRC | RCC_SYSPLL_GATE | RCC_SYSPLLDIV)) | RCC_SYSPLLSRC_PLL;
    RCC->CTLR |= RCC_PLLON;
    while ((RCC->CTLR & RCC_PLLRDY) == 0u) {
    }
    RCC->CFGR2 |= RCC_SYSPLL_GATE;
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_PLL;
    while ((RCC->CFGR0 & RCC_SWS) != RCC_SWS_PLL) {
    }
}

/* ------------------------------------------------------------------------------------------------ init */
void hw_full_init(void)
{
    clock_200();
    clock_khz = MCU_HCLK_HZ / 1000u * (1000u - MCU_HSI_BOOT_TRIM_STEPS * MCU_HSI_TRIM_PPM_STEP / 1000u) / 1000u;
    ms_count = tick_rem = 0;
    RCC->HBPCENR |= RCC_DMA1EN | RCC_CRCEN;
    RCC->PB2PCENR |= RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_SPI1EN;
    RCC->PB1PCENR |= RCC_TIM3EN | RCC_PWREN;

    /* free-running 32-bit SysTick at HCLK */
    SysTick->CMP = 0xFFFFFFFFu;
    SysTick->CTLR = (1u << 5) /* INIT */ | (1u << 2) /* HCLK */ | 1u;
    tick_last = hw_ticks();
    hw_kick();
    PWR->CTLR = (PWR->CTLR & ~PWR_CTLR_PLS) | PWR_CTLR_PLS_2V9 | PWR_CTLR_PVDE;

    /* remaps: SPI1 -> PA15 / PB3 / PB4 / PB5 (NSS by software), TIM3 CH1 -> PB4 (RM p.132-135); USART7 TX -> PA9
     * (remap 11, RM p.138), the status answers; PA9 stays a floating input outside an answer */
    AFIO->PCFR1 = (AFIO->PCFR1 & ~(AFIO_PCFR1_TIM3_RM | AFIO_PCFR1_TIM2_RM)) | AFIO_PCFR1_SPI1_RM | AFIO_PCFR1_TIM3_RM_PB4;
    AFIO->PCFR2 = (AFIO->PCFR2 & ~AFIO_PCFR2_USART7_RM) | AFIO_PCFR2_USART7_RM_PA9;
    RCC->PB1PCENR |= RCC_USART7EN;
    /* the strip held LOW, mux direct, replay selected, input listening, LED off */
    GPIOA->BCR = (1u << 8) | (1u << 11) | (1u << 4);
    pin_cfg(GPIOA, 8, CFG_OUT_PP);
    pin_cfg(GPIOA, 11, CFG_OUT_2M);
    pin_cfg(GPIOA, 4, CFG_OUT_PP);
    GPIOB->BCR = (1u << 6) | (1u << 7);
    pin_cfg(GPIOB, 6, CFG_OUT_2M);
    pin_cfg(GPIOB, 7, CFG_OUT_2M);
    pin_cfg(GPIOB, 4, CFG_IN_FLOAT);
    pin_cfg(GPIOB, 3, CFG_AF_PP);
    /* the gate (upd_forward.h): TIM2 CH4 on FWD_EN, closed; bl_main opens it at the first quiet LOW (opening in a frame
     * tail would expose a truncated frame downstream; the entry supplies a >= 50 ms LOW) */
    upd_forward_init();

    /* edge counter: TIM3 CH1 captures RO's falling edges (PB4, filter N = 2), TRGO per capture clocks TIM2 (ITR2) */
    TIM3->PSC = 0;
    TIM3->ATRLR = 0xFFFFu;
    TIM3->CHCTLR1 = (uint16_t)(1u | (1u << 4));
    TIM3->CCER = TIM_CC1E | TIM_CC1P;
    TIM3->CTLR2 = (uint16_t)(3u << 4);
    TIM3->CTLR1 = TIM_CEN;

    /* DIN sampler: SPI1 master, receive only, 16-bit frames, PCLK2 / 2 = 50 MHz with HSRXEN (RM p.358, 364), DMA1 ch2
     * circular (RM p.147). BENCH: the RX-only clock runs continuously */
    SPI1->I2SCFGR = 0;
    SPI1->CTLR1 = SPI_CTLR1_MSTR | SPI_CTLR1_SSM | SPI_CTLR1_SSI | SPI_CTLR1_RXONLY | SPI_CTLR1_DFF;
    SPI1->HSCR = SPI_HSCR_HSRXEN;
    SPI1->CTLR2 = SPI_CTLR2_RXDMAEN;
    DMA1_Channel2->CFGR = 0;
    DMA1_Channel2->PADDR = (uint32_t)&SPI1->DATAR;
    DMA1_Channel2->MADDR = (uint32_t)hw_ring;
    DMA1_Channel2->CNTR = HW_RING_WORDS;
    DMA1_Channel2->CFGR = DMA_CFGR1_CIRC | DMA_CFGR1_MINC | DMA_CFGR1_PSIZE_0 | DMA_CFGR1_MSIZE_0 |
                          DMA_CFGR1_PL_0 | DMA_CFGR1_PL_1 | DMA_CFGR1_EN;
    SPI1->CTLR1 |= SPI_CTLR1_SPE;
}

/* ------------------------------------------------------------------------------------------------ DIN, gate */
uint32_t hw_ring_pos(void)
{
    return (HW_RING_WORDS - DMA1_Channel2->CNTR) & (HW_RING_WORDS - 1u);
}

uint16_t hw_edges(void)
{
    return (uint16_t)gate_edges();
}

/* The update forward path (forced HIGH) stays open, including through blocking flash work. */
void hw_gate_open(void)
{
    upd_forward_set(1);
    TIM2->CNT = 0;
}

/* the slot plan: closed for this board's 256 bits, then open (the boards behind keep their slots) */
void hw_gate_ultra(void)
{
    gate_slot_arm();
}

/* ------------------------------------------------------------------------------------------------ status answer */
/* docs/timing.md, marker port: the receiver drives only after the hub released the line (by frame end + 250 us,
 * + 20 us), enables by frame end + 900 us, sends one 0xFF lead byte and the 32-byte answer 8N1 with inter-byte gaps
 * under 8 us (TXE polled, interrupts off), and drops DE within 5 us after TC. BLANK_N holds the strip LOW meanwhile. */
__attribute__((noipa)) int hw_reply(const uint8_t *a, uint32_t at, uint16_t brr, uint32_t t0)
{
    uint32_t i, t, high_since = 0;
    uint8_t high = 0;
    /* `at` is t0 + 450 host us at the measured rate. Use that same scale,
     * including across SysTick wrap; never infer a release from a late HIGH. */
    const uint32_t span = at - t0;
    const uint32_t release_by = t0 + (uint32_t)((uint64_t)span * 250u / 450u);
    const uint32_t guard = (uint32_t)(((uint64_t)span * 20u + 449u) / 450u);
    USART7->BRR = brr;
    USART7->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE;
    if ((int32_t)(hw_ticks() - at) > 0) {
        return -1;                            /* late: no part of a burst */
    }
    for (;;) {
        t = hw_ticks();
        if (!(GPIOB->INDR & (1u << 4))) {
            high = 0;
            if ((int32_t)(t - release_by) >= 0) return -1;
        } else if (!high) {
            if ((int32_t)(t - release_by) > 0) return -1;
            high = 1;
            high_since = t;
        }
        if ((int32_t)(t - at) >= 0) break;
    }
    if (!high || t - high_since < guard) return -1;
    GPIOA->BSHR = 1u << 11;                   /* DE high: the line HIGH */
    pin_cfg(GPIOA, 9, CFG_AF_PP);             /* TX drives (idles HIGH) */
    t = hw_ticks();
    while (hw_ticks() - t < 3u * HW_TICKS_US) {   /* >= 2 us before the first start bit */
    }
    USART7->DATAR = 0xFFu;
    for (i = 0; i < 32u; i++) {
        while (!(USART7->STATR & USART_STATR_TXE)) {
        }
        USART7->DATAR = a[i];
    }
    while (!(USART7->STATR & USART_STATR_TC)) {
    }
    GPIOA->BCR = 1u << 11;                    /* DE low: the hub's line again */
    pin_cfg(GPIOA, 9, CFG_IN_FLOAT);
    return 0;
}

/* ------------------------------------------------------------------------------------------------ misc */
void hw_kick(void)
{
    IWDG->CTLR = 0xAAAAu;
}

void hw_led(int on)
{
    GPIOA->BSHR = on ? 1u << 4 : 1u << (4 + 16);         /* status LED (PA4, active high) */
}

void bl_plat_kick(void)
{
    hw_kick();
}

void bl_plat_watchdog_start(void)
{
    /* LSI 25-60 kHz / 64, reload 2500: 2.7-6.4 s, including early app startup (RM p.52-55, DS p.66) */
    IWDG->CTLR = 0x5555u;
    IWDG->PSCR = 4u;
    IWDG->RLDR = 2500u;
    IWDG->CTLR = 0xAAAAu;
    IWDG->CTLR = 0xCCCCu;
    RCC->PB1PCENR |= RCC_PWREN;
    PWR->CTLR = (PWR->CTLR & ~PWR_CTLR_PLS) | PWR_CTLR_PLS_2V9 | PWR_CTLR_PVDE;
}

int bl_plat_din_high(void)
{
    RCC->PB2PCENR |= RCC_IOPBEN;              /* PB4 is a floating input after reset */
    return (GPIOB->INDR & (1u << 4)) != 0u;
}

uint32_t bl_plat_boot_ticks(void)
{
    if (!(SysTick->CTLR & 1u)) {
        SysTick->CMP = 0xFFFFFFFFu;
        SysTick->CTLR = (1u << 5) | (1u << 2) | 1u;
    }
    return hw_ticks();
}

int bl_plat_power_ok(void)
{
    return (PWR->CSR & PWR_CSR_PVDO) == 0u;
}

/* ------------------------------------------------------------------------------------------------ flash */
/* RM 32.5 (p.586-593): standard halfword programming (PG) and 4 KB sector erase (PER) after the KEYR unlock; the fast
 * 256-byte page program (FTPG, 64 words each when WRBSY is clear, then PGSTRT) after MODEKEYR as well. The flash
 * access clock stays 100 MHz (no clock change). The waits run in the zero-wait flash against the deadline. */
#define FLASH_KEY1   0x45670123u
#define FLASH_KEY2   0xCDEF89ABu

const uint8_t *bl_plat_ptr(uint32_t addr)
{
    return (const uint8_t *)addr;
}

static int fl_unlock(uint32_t fast)
{
    int32_t left = (int32_t)(deadline_us - physical_us());
    if (left <= 0 || !bl_plat_power_ok()) return -1;
    slow_budget = (uint32_t)left / 1000u * clock_khz + (uint32_t)left % 1000u * clock_khz / 1000u;
    slow_start = hw_ticks();
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    if (fast) {
        FLASH->MODEKEYR = FLASH_KEY1;
        FLASH->MODEKEYR = FLASH_KEY2;
    }
    FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;
    return 0;
}

/* (RM V1.2 p.579, RM-CN p.512: functions related to flash erase and programming only in the zero-wait flash: these
 * are .text, inside the protected boot sectors at 0x08000000. BENCH: execution and its fetches during BSY) */
static __attribute__((noinline)) void fl_wait(uint32_t mask)
{
    while (FLASH->STATR & mask) {
        if (SysTick->CNT - slow_start >= slow_budget) {
            NVIC_SystemReset();
            for (;;) {}
        }
    }
}

static __attribute__((noinline)) int fl_done(uint32_t ctlr_bits)
{
    uint32_t st;
    fl_wait(FLASH_STATR_BSY | FLASH_STATR_WRBSY);
    st = FLASH->STATR;
    if (SysTick->CNT - slow_start >= slow_budget) st |= FLASH_STATR_WRPRTERR;
    FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;
    FLASH->CTLR &= ~ctlr_bits;
    FLASH->CTLR |= FLASH_CTLR_LOCK | FLASH_CTLR_FLOCK;
    hw_flash_stall = 1;                       /* (the sampler ran on, the decoder did not: resync) */
    return (st & FLASH_STATR_WRPRTERR) || !(st & FLASH_STATR_EOP) ? -1 : 0;
}

/* one minimum erase unit: the 4 KB sector at addr (layout 4) */
__attribute__((noinline)) int bl_plat_erase_page(uint32_t addr)
{
    if (fl_unlock(0)) return -1;
    FLASH->CTLR |= FLASH_CTLR_PER;
    FLASH->ADDR = addr;
    FLASH->CTLR |= FLASH_CTLR_STRT;
    return fl_done(FLASH_CTLR_PER);
}

/* RM p.589-590 (with the CN corrections: wait WRBSY = 0, start with PGSTRT). BENCH: sequence, time */
__attribute__((noinline)) int bl_plat_program_page(uint32_t addr, const uint8_t *d)
{
    uint32_t i;
    if (fl_unlock(1)) return -1;
    fl_wait(FLASH_STATR_BSY);
    FLASH->CTLR |= FLASH_CTLR_FTPG;
    for (i = 0; i < 64u; i++) {
        const uint8_t *p = d + 4u * i;
        *(volatile uint32_t *)(addr + 4u * i) =
            (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        fl_wait(FLASH_STATR_WRBSY);
    }
    FLASH->CTLR |= FLASH_CTLR_PGSTRT;
    return fl_done(FLASH_CTLR_FTPG);
}

__attribute__((noinline)) int bl_plat_program_half(uint32_t addr, uint16_t v)
{
    if (fl_unlock(0)) return -1;
    FLASH->CTLR |= FLASH_CTLR_PG;
    *(volatile uint16_t *)addr = v;
    return fl_done(FLASH_CTLR_PG);
}

#define BL_OB_ADDR 0x1FFFF800u

/* writable boot sectors, or the wrong memory split: the protection write is due (RM p.585, 591) */
int bl_plat_boot_open(void)
{
    return (FLASH->WPR & 0xFu) != 0u || (FLASH->OBR & FLASH_OBR_RAM_CODE_MOD) != 0u;
}

int bl_plat_layout_ok(void)
{
    /* layout 4: sectors 0-3 protected, every other sector writable, the 512 KB / 200 KB split */
    return options_trusted && FLASH->WPR == 0xFFFFFFF0u && (FLASH->OBR & FLASH_OBR_RAM_CODE_MOD) == 0u;
}

int bl_plat_options_read(uint16_t ob[8])
{
    const volatile uint16_t *p = (const volatile uint16_t *)BL_OB_ADDR;
    uint32_t i, ok = 1;
    for (i = 0; i < 8u; i++) {
        uint32_t a = p[i];
        ok &= a == p[i] && ((a ^ (a >> 8)) & 0xFFu) == 0xFFu;
        ob[i] = (uint16_t)(a & 0xFFu);
    }
    options_trusted = (uint8_t)(ok && ob[0] == 0xa5u);
    return (int)ok;
}

static int ob_wait(void)
{
    uint32_t st;
    fl_wait(FLASH_STATR_BSY);
    st = FLASH->STATR;
    FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;
    hw_kick();
    return (st & FLASH_STATR_WRPRTERR) || !(st & FLASH_STATR_EOP) ? -1 : 0;
}

/* erase + all eight bytes again (an erase leaves 0x39 everywhere: read-protected until rewritten, RM p.592) */
int bl_plat_options_write(const uint16_t ob[8])
{
    uint32_t i;
    int rc = -1;
    bl_plat_begin(2u * MCU_UPD_ERASE_MS);
    if (fl_unlock(0)) return -1;
    FLASH->OBKEYR = FLASH_KEY1;
    FLASH->OBKEYR = FLASH_KEY2;
    if (!(FLASH->CTLR & FLASH_CTLR_LOCK) && (FLASH->CTLR & FLASH_CTLR_OBWRE)) {
        FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;
        FLASH->CTLR |= FLASH_CTLR_OBER;
        FLASH->CTLR |= FLASH_CTLR_STRT;
        rc = ob_wait();
        FLASH->CTLR &= ~(FLASH_CTLR_OBER | FLASH_CTLR_STRT);
        FLASH->CTLR |= FLASH_CTLR_OBPG;
        for (i = 0; i < 8u && rc == 0; i++) {
            volatile uint16_t *p = (volatile uint16_t *)(BL_OB_ADDR + 2u * i);
            *p = ob[i];
            rc = ob_wait();
            if ((*p & 0xFFu) != ob[i]) {
                rc = -1;
            }
        }
    }
    FLASH->CTLR = (FLASH->CTLR & ~(FLASH_CTLR_OBPG | FLASH_CTLR_OBWRE)) | FLASH_CTLR_LOCK | FLASH_CTLR_FLOCK;
    return rc;
}
