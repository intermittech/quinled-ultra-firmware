/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V203 bootloader backend. DIN sampling runs at 36 MHz; flash waits execute
 * from SRAM with monotonic deadlines. Output pins retain their safe update state.
 * Register sequences and declared timing still require physical qualification. */
#include "ch32v20x.h"
#include "../../../bootloader/bl_hw.h"
#include "../../../bootloader/bl_core.h"
#include "boot_api_layout.h"

uint16_t hw_ring[HW_RING_WORDS] __attribute__((aligned(4), section(".noinit")));
volatile uint8_t hw_flash_stall;
static uint32_t ahb_at_reset;
static uint32_t clock_mhz = 8u, deadline_us, slow_start, slow_budget;
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
#define CFG_AF_PP    0xBu
#define OCM_TOGGLE   3u
#define OCM_FORCE_LO 4u
#define OCM_FORCE_HI 5u

static void pin_cfg(GPIO_TypeDef *g, uint32_t pin, uint32_t cfg)
{
    volatile uint32_t *r = pin < 8u ? &g->CFGLR : &g->CFGHR;
    uint32_t sh = (pin & 7u) * 4u;
    *r = (*r & ~(0xFu << sh)) | (cfg << sh);
}

/* ------------------------------------------------------------------------------------------------ reset / boot */
#if BL_HW_ID == 3
/* Hold the pixel output low before choosing a boot path. The app retains this
 * state until its normal receive path is ready. */
void hw_blank_early(void)
{
    RCC->APB2PCENR |= RCC_IOPAEN;
    GPIOA->BCR = 1u << 8;
    pin_cfg(GPIOA, 8, CFG_OUT_PP);
}
#endif
uint32_t hw_reset_flags_take(void)
{
    uint32_t f = RCC->RSTSCKR;
    RCC->RSTSCKR |= RCC_RMVF;                 /* our own software resets would pile up otherwise */
    return f;
}

void hw_crc_on(void)
{
    ahb_at_reset = RCC->AHBPCENR;
    RCC->AHBPCENR |= RCC_CRCEN;
}

void hw_crc_off(void)
{
    RCC->AHBPCENR = ahb_at_reset;
}

uint32_t bl_plat_crc32(uint32_t addr, uint32_t nwords)
{
    const volatile uint32_t *p = (const volatile uint32_t *)addr;
    CRC->CTLR = 1u;                           /* reset to 0xFFFFFFFF */
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

/* The descriptor's entry is physical (inside the body, as its bounds are checked); the app is linked at the zero alias
 * and its startup computes gp, sp and its RAM PC-relatively, so the jump uses the alias (RM V2.5 p.16: flash at both). */
void hw_jump_app(void)
{
    const uint32_t entry = upd_get32(bl_plat_ptr(UPD_APP_DESC) + UPD_D_ENTRY);
    SysTick->CTLR = 0;                       /* the optional power-on DIN qualification used HSI ticks */
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
    return (uint32_t)SysTick->CNT;            /* low word */
}

uint32_t hw_ms(void)
{
    uint32_t now = hw_ticks(), d = now - tick_last + tick_rem;
    tick_last = now;
    ms_count += d / (clock_mhz * 1000u);
    tick_rem = d % (clock_mhz * 1000u);
    return ms_count;
}

uint32_t bl_plat_ms(void) { return hw_ms(); }

static uint32_t physical_us(void)
{
    uint32_t ms = hw_ms();
    return ms * 1000u + tick_rem / clock_mhz;
}

void bl_plat_begin(uint32_t deadline_ms)
{
    (void)bl_plat_boot_ticks();
    /* Reserve 100 us inside the declared deadline for failure/reset cleanup. */
    deadline_us = physical_us() + deadline_ms * 1000u - 100u;
    hw_kick();
}

static void clock_144(void)
{
    RCC->CTLR |= RCC_HSION;
    RCC->CFGR0 &= ~(RCC_SW | RCC_HPRE | RCC_PPRE1 | RCC_PPRE2 | RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL);
    EXTEN->EXTEN_CTR |= EXTEN_PLL_HSI_PRE;    /* HSI undivided into the PLL */
    RCC->CFGR0 |= RCC_PLLSRC_HSI_Div2 | RCC_PLLMULL18;
    RCC->CTLR |= RCC_PLLON;
    while ((RCC->CTLR & RCC_PLLRDY) == 0u) {
    }
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_PLL;
    while ((RCC->CFGR0 & RCC_SWS) != 0x08u) {
    }
}

/* ------------------------------------------------------------------------------------------------ init */
void hw_full_init(void)
{
    clock_144();
    clock_mhz = MCU_HCLK_HZ / 1000000u;
    ms_count = tick_rem = 0;
    RCC->AHBPCENR |= RCC_DMA1EN | RCC_CRCEN;
    RCC->APB2PCENR |= RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_SPI1EN;
    RCC->APB1PCENR |= RCC_TIM2EN | RCC_TIM3EN | RCC_PWREN;

    /* free-running 64-bit SysTick at HCLK (counts up from 0) */
    SysTick->CMP = 0xFFFFFFFFFFFFFFFFull;
    SysTick->CTLR = (1u << 5) /* INIT */ | (1u << 2) /* HCLK */ | 1u;
    tick_last = hw_ticks();

    /* bl_boot started the independent watchdog before choosing any path. */
    hw_kick();

    /* PVD: flash is only written while VDD >= ~2.9 V */
    PWR->CTLR = (PWR->CTLR & ~PWR_CTLR_PLS) | PWR_CTLR_PLS_2V9 | PWR_CTLR_PVDE;

    /* Default pin routing: TIM2_CH2 on FWD_EN (below, once the timer holds it LOW), TIM3_CH1 on DIN. */
    GPIOA->BSHR = 1u << (1 + 16);                        /* FWD_EN low until the timer drives it */
    GPIOB->BSHR = 1u << (15 + 16);                       /* status LED off */
    pin_cfg(GPIOA, 0, 0x0u);                             /* analog: unused here */
    pin_cfg(GPIOA, 4, 0x0u);
    pin_cfg(GPIOA, 6, CFG_IN_FLOAT);
    pin_cfg(GPIOA, 5, CFG_AF_PP);
#if BL_HW_ID == 3
    /* Hold the strip blank through boot reception; DE_OUT keeps its reset input state (the port-2 driver on). */
    GPIOA->BCR = 1u << 8;
    pin_cfg(GPIOA, 8, CFG_OUT_PP);
    GPIOB->BCR = (1u << 1) | (1u << 10);
    pin_cfg(GPIOB, 1, CFG_OUT_PP);       /* MUX direct */
    pin_cfg(GPIOB, 10, CFG_OUT_PP);      /* input DE low: listen */
#endif
    pin_cfg(GPIOB, 15, CFG_OUT_PP);

    /* gate: TIM3 CH1 captures DIN falling edges on PA6 (filter fCK, N = 2); its trigger output pulses on each capture
     * (MMS = 011) and clocks TIM2 (external clock mode 1 on ITR2 = TIM3), so TIM2->CNT counts DIN edges, as in the
     * app; TIM2 CH2 drives FWD_EN */
    TIM3->PSC = 0;
    TIM3->ATRLR = 0xFFFFu;
    TIM3->CHCTLR1 = (uint16_t)(1u /* CC1S = TI1 */ | (1u << 4) /* IC1F: fCK, N = 2 */);
    TIM3->CCER = TIM_CC1E | TIM_CC1P;
    TIM3->CTLR2 = (uint16_t)(3u << 4);                   /* MMS = 011: compare pulse, also on capture */
    TIM3->CTLR1 = TIM_CEN;
    TIM2->PSC = 0;
    TIM2->ATRLR = 0xFFFFu;
    TIM2->SMCFGR = (uint16_t)((2u << 4) /* TS = ITR2 (TIM3) */ | 7u /* SMS: external clock mode 1 */);
    TIM2->CHCTLR1 = (uint16_t)(OCM_FORCE_LO << 12);      /* closed: bl_main opens it at the first quiet LOW */
    TIM2->CCER = TIM_CC2E;
    TIM2->CTLR1 = TIM_CEN;
    pin_cfg(GPIOA, 1, CFG_AF_PP);                        /* every board: the forced output survives flash work */

    /* DIN sampler: SPI1 master, receive only, 16-bit frames, 144 MHz / 4 = 36 MHz, DMA1 ch2 circular.
     * BENCH (as the app): the RX-only clock runs continuously */
    SPI1->CTLR1 = SPI_CTLR1_MSTR | (1u << 3) | SPI_CTLR1_SSM | SPI_CTLR1_SSI | SPI_CTLR1_RXONLY | SPI_CTLR1_DFF;
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
    return (uint16_t)TIM2->CNT;
}

static void gate_mode(uint32_t ocm)
{
    TIM2->CHCTLR1 = (uint16_t)((TIM2->CHCTLR1 & ~(7u << 12)) | (ocm << 12));
}

void hw_gate_open(void)
{
    gate_mode(OCM_FORCE_HI);
    TIM2->CNT = 0;
}

void hw_gate_ultra(void)
{
    TIM2->CNT = 0;
    gate_mode(OCM_FORCE_LO);
    TIM2->CH2CVR = 256u;                      /* toggles to open at the 256th falling edge (the slot's last bit) */
    gate_mode(OCM_TOGGLE);
}

/* ------------------------------------------------------------------------------------------------ misc */
void hw_kick(void)
{
    IWDG->CTLR = 0xAAAAu;
}

void hw_led(int on)
{
    GPIOB->BSHR = on ? 1u << 15 : 1u << (15 + 16);       /* status LED (PB15, active high) */
}

void bl_plat_kick(void)
{
    hw_kick();
}

void bl_plat_watchdog_start(void)
{
    /* LSI 25-60 kHz / 64, reload 2500: 2.7-6.4 s, including early app startup. */
    IWDG->CTLR = 0x5555u;
    IWDG->PSCR = 4u;
    IWDG->RLDR = 2500u;
    IWDG->CTLR = 0xAAAAu;
    IWDG->CTLR = 0xCCCCu;
    /* Enable PVD before reading the descriptor/options: it has time to settle before an option write. */
    RCC->APB1PCENR |= RCC_PWREN;
    PWR->CTLR = (PWR->CTLR & ~PWR_CTLR_PLS) | PWR_CTLR_PLS_2V9 | PWR_CTLR_PVDE;
}

int bl_plat_din_high(void)
{
    RCC->APB2PCENR |= RCC_IOPAEN;             /* PA6 is already a floating input after reset */
    return (GPIOA->INDR & (1u << 6)) != 0u;
}

uint32_t bl_plat_boot_ticks(void)
{
    if (!(SysTick->CTLR & 1u)) {
        SysTick->CMP = 0xFFFFFFFFFFFFFFFFull;
        SysTick->CTLR = (1u << 5) | (1u << 2) | 1u;
    }
    return hw_ticks();
}

int bl_plat_power_ok(void)
{
    return (PWR->CSR & PWR_CSR_PVDO) == 0u;   /* PVDO = 1: VDD below the threshold */
}

/* ------------------------------------------------------------------------------------------------ flash */
#define FLASH_KEY1   0x45670123u
#define FLASH_KEY2   0xCDEF89ABu
#define STATR_WRBSY  0x02u                    /* fast page programming: data register busy */

const uint8_t *bl_plat_ptr(uint32_t addr)
{
    return (const uint8_t *)addr;
}

/* HCLK / 2 around every flash operation (as the app's flash_program; no interrupts here, nothing else to pause) */
static int fl_unlock(void)
{
    int32_t left = (int32_t)(deadline_us - physical_us());
    if (left <= 0 || !bl_plat_power_ok()) return -1;
    slow_budget = (uint32_t)left * (clock_mhz / 2u);
    slow_start = hw_ticks();
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | RCC_HPRE_DIV2;
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    FLASH->MODEKEYR = FLASH_KEY1;             /* fast mode: 256-byte page erase / program */
    FLASH->MODEKEYR = FLASH_KEY2;
    FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;
    return 0;
}

/* SRAM execution keeps the deadline observable while flash is busy. A stuck
 * operation resets at the declared deadline with the outputs still safe. */
static __attribute__((section(".ramtext"), noinline)) void fl_wait(uint32_t mask)
{
    while (FLASH->STATR & mask) {
        if ((uint32_t)SysTick->CNT - slow_start >= slow_budget) {
            NVIC_SystemReset();
            for (;;) {}
        }
    }
}

static __attribute__((section(".ramtext"), noinline)) int fl_done(uint32_t ctlr_bits)
{
    uint32_t st;
    fl_wait(FLASH_STATR_BSY | STATR_WRBSY);
    st = FLASH->STATR;
    if (hw_ticks() - slow_start >= slow_budget) st |= FLASH_STATR_WRPRTERR;
    FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;   /* write 1 to clear */
    FLASH->CTLR &= ~ctlr_bits;
    FLASH->CTLR |= FLASH_CTLR_LOCK | FLASH_CTLR_FAST_LOCK;
    /* SysTick counted half as fast while HCLK was divided. */
    tick_rem += hw_ticks() - slow_start;
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | RCC_HPRE_DIV1;
    hw_flash_stall = 1;
    return (st & FLASH_STATR_WRPRTERR) || !(st & FLASH_STATR_EOP) ? -1 : 0;
}

__attribute__((section(".ramtext"), noinline)) int bl_plat_erase_page(uint32_t addr)
{
    if (fl_unlock()) return -1;
    FLASH->CTLR |= FLASH_CTLR_PAGE_ER;
    FLASH->ADDR = addr;
    FLASH->CTLR |= FLASH_CTLR_STRT;
    return fl_done(FLASH_CTLR_PAGE_ER);
}

/* RM 32.5.6: FTPG, 64 word writes (each when WRBSY is clear), PGSTRT, wait for BSY. BENCH: sequence, time */
__attribute__((section(".ramtext"), noinline)) int bl_plat_program_page(uint32_t addr, const uint8_t *d)
{
    uint32_t i;
    if (fl_unlock()) return -1;
    FLASH->CTLR |= FLASH_CTLR_PAGE_PG;
    fl_wait(FLASH_STATR_BSY | STATR_WRBSY);
    for (i = 0; i < 64u; i++) {
        const uint8_t *p = d + 4u * i;
        *(volatile uint32_t *)(addr + 4u * i) =
            (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        fl_wait(STATR_WRBSY);
    }
    FLASH->CTLR |= FLASH_CTLR_PG_STRT;
    return fl_done(FLASH_CTLR_PAGE_PG);
}

/* standard half-word programming (descriptor, status records). BENCH: into a page programmed record by record */
__attribute__((section(".ramtext"), noinline)) int bl_plat_program_half(uint32_t addr, uint16_t v)
{
    if (fl_unlock()) return -1;
    FLASH->CTLR |= FLASH_CTLR_PG;
    *(volatile uint16_t *)addr = v;
    return fl_done(FLASH_CTLR_PG);
}

#define BL_OB_ADDR 0x1FFFF800u

int bl_plat_boot_open(void)
{
    return (FLASH->WPR & 3u) != 0u;
}

int bl_plat_layout_ok(void)
{
    /* This V203 layout is fixed: 64 KiB code / 20 KiB SRAM, no selectable
     * V407 memory split. Only the bootloader's first two 4 KiB units are protected. */
    return options_trusted && (FLASH->WPR & 0xffffu) == 0xfffcu;
}

int bl_plat_options_read(uint16_t ob[8])
{
    const volatile uint16_t *p = (const volatile uint16_t *)BL_OB_ADDR;
    uint32_t i, ok = 1;
    /* Called ONLY at HSI 8 MHz, before hw_full_init: no unsafe 144 MHz option reads. */
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

int bl_plat_options_write(const uint16_t ob[8])
{
    uint32_t i;
    int rc = -1;
    bl_plat_begin(2u * UPD_V203_ERASE_MS);
    if (fl_unlock()) return -1;
    FLASH->OBKEYR = FLASH_KEY1;
    FLASH->OBKEYR = FLASH_KEY2;
    if (!(FLASH->CTLR & FLASH_CTLR_LOCK) && (FLASH->CTLR & FLASH_CTLR_OPTWRE)) {
        FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;
        FLASH->CTLR |= FLASH_CTLR_OPTER;
        FLASH->CTLR |= FLASH_CTLR_STRT;
        rc = ob_wait();
        FLASH->CTLR &= ~(FLASH_CTLR_OPTER | FLASH_CTLR_STRT);
        FLASH->CTLR |= FLASH_CTLR_OPTPG;
        for (i = 0; i < 8u && rc == 0; i++) {
            volatile uint16_t *p = (volatile uint16_t *)(BL_OB_ADDR + 2u * i);
            *p = ob[i];                      /* hardware supplies the inverse */
            rc = ob_wait();
            if ((*p & 0xFFu) != ob[i]) {
                rc = -1;
            }
        }
    }
    FLASH->CTLR = (FLASH->CTLR & ~(FLASH_CTLR_OPTPG | FLASH_CTLR_OPTWRE)) |
                  FLASH_CTLR_LOCK | FLASH_CTLR_FAST_LOCK;
    tick_rem += hw_ticks() - slow_start;
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | RCC_HPRE_DIV1;
    return rc;
}
