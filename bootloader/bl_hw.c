/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Bootloader hardware layer for the CH32V203F8U6 (QuinLED dig2analog-ultra-5ch). Register sequences follow the
 * reference manual CH32FV2x_V3xRM V2.5 and the app's hw.c; items marked BENCH need the real chip (README.md).
 *
 * Clock: HSI x 18 = 144 MHz like the app, HCLK = PCLK1 = PCLK2, so the DIN sampler runs at 36 MHz with the app's
 * decoder thresholds (bootloader/measure_bl.py: ~61 % CPU inside a frame). Flash operations run at HCLK / 2 (the
 * flash access clock must stay <= 60 MHz; the app's sequence, RM 32.1 note 2); the main loop resyncs the decoder
 * after each one, since the sampler ran at half speed meanwhile.
 * Pins (rev 0.4, the app's pin map): PA6 DIN = SPI1_MISO (sampler, 36 MHz; SCK PA5, pad NC) and TIM3_CH1, whose
 * capture trigger clocks TIM2 (ITR2): the edge counter; PA1 FWD_EN = TIM2_CH2 (gate); PA9 TIM1_CH2 = PWM_R
 * (feedback, 10 %); PB15 D9; PA2 USART2_TX (TP5); PA0 / PA4 analog (unused here). All other PWM pins stay inputs: the
 * board's pull-downs keep their gate drivers off. */
#include "ch32v20x.h"
#include "bl_hw.h"
#include "bl_core.h"

uint16_t hw_ring[HW_RING_WORDS] __attribute__((aligned(4), section(".noinit")));
volatile uint8_t hw_flash_stall;
static uint32_t ahb_at_reset;

/* the compiler may call these for struct copies / clearing loops (no C library is linked) */
void *memcpy(void *d, const void *s, unsigned int n)
{
    uint8_t *a = (uint8_t *)d;
    const uint8_t *b = (const uint8_t *)s;
    while (n--) {
        *a++ = *b++;
    }
    return d;
}

void *memset(void *d, int c, unsigned int n)
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
#define OCM_PWM1     6u

static void pin_cfg(GPIO_TypeDef *g, uint32_t pin, uint32_t cfg)
{
    volatile uint32_t *r = pin < 8u ? &g->CFGLR : &g->CFGHR;
    uint32_t sh = (pin & 7u) * 4u;
    *r = (*r & ~(0xFu << sh)) | (cfg << sh);
}

/* ------------------------------------------------------------------------------------------------ reset / boot */
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

void hw_jump_app(void)
{
    hw_crc_off();
    ((void (*)(void))UPD_APP_ENTRY)();        /* the app's startup sets sp, gp, mtvec, interrupts and its clock */
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
    ms_count += d / 144000u;
    tick_rem = d % 144000u;
    return ms_count;
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
    RCC->AHBPCENR |= RCC_DMA1EN | RCC_CRCEN;
    RCC->APB2PCENR |= RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_TIM1EN | RCC_SPI1EN;
    RCC->APB1PCENR |= RCC_TIM2EN | RCC_TIM3EN | RCC_PWREN | RCC_USART2EN;

    /* free-running 64-bit SysTick at HCLK (counts up from 0) */
    SysTick->CMP = 0xFFFFFFFFFFFFFFFFull;
    SysTick->CTLR = (1u << 5) /* INIT */ | (1u << 2) /* HCLK */ | 1u;
    tick_last = hw_ticks();

    /* independent watchdog: LSI 25-60 kHz / 64, reload 2500 -> 2.7-6.4 s (typ. 4 s); kicked by the main loop and
     * between flash operations (the longest single one is a 16 ms erase) */
    IWDG->CTLR = 0x5555u;
    IWDG->PSCR = 4u;
    IWDG->RLDR = 2500u;
    IWDG->CTLR = 0xAAAAu;
    IWDG->CTLR = 0xCCCCu;

    /* PVD: flash is only written while VDD >= ~2.9 V */
    PWR->CTLR = (PWR->CTLR & ~PWR_CTLR_PLS) | PWR_CTLR_PLS_2V9 | PWR_CTLR_PVDE;

    /* pins (no AFIO remap: TIM2_CH2 on PA1, TIM3_CH1 on PA6, TIM1_CH2 on PA9, USART2_TX on PA2) */
    GPIOA->BSHR = 1u << (1 + 16);                        /* FWD_EN low until the timer drives it */
    GPIOB->BSHR = 1u << (15 + 16);                       /* D9 off */
    pin_cfg(GPIOA, 0, 0x0u);                             /* analog: unused here */
    pin_cfg(GPIOA, 4, 0x0u);
    pin_cfg(GPIOA, 6, CFG_IN_FLOAT);
    pin_cfg(GPIOA, 5, CFG_AF_PP);
    pin_cfg(GPIOA, 1, CFG_AF_PP);
    pin_cfg(GPIOA, 2, CFG_AF_PP);
    pin_cfg(GPIOA, 9, CFG_AF_PP);
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
    TIM2->CHCTLR1 = (uint16_t)(OCM_FORCE_HI << 12);      /* open: forward everything */
    TIM2->CCER = TIM_CC2E;
    TIM2->CTLR1 = TIM_CEN;

    /* feedback: TIM1 CH2 (PWM_R) 1 kHz, 10 % when on */
    TIM1->PSC = 143u;
    TIM1->ATRLR = 999u;
    TIM1->CHCTLR1 = (uint16_t)((OCM_PWM1 << 12) | TIM_OC2PE);
    TIM1->CH2CVR = 0;
    TIM1->CCER = TIM_CC2E;
    TIM1->BDTR = TIM_MOE;
    TIM1->CTLR1 = TIM_ARPE | TIM_CEN;

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

    /* debug UART: USART2 TX only, 115200 8N1 */
    USART2->BRR = (uint16_t)(144000000u / 115200u);
    USART2->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE;
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
void hw_feedback(int on)
{
    TIM1->CH2CVR = on ? 100u : 0u;
    GPIOB->BSHR = on ? (1u << 15) : (1u << (15 + 16)); /* D9 on PB15 */
}

void hw_kick(void)
{
    IWDG->CTLR = 0xAAAAu;
}

void bl_plat_kick(void)
{
    hw_kick();
}

int bl_plat_power_ok(void)
{
    return (PWR->CSR & PWR_CSR_PVDO) == 0u;   /* PVDO = 1: VDD below the threshold */
}

void hw_puts(const char *s)
{
    while (*s) {
        while ((USART2->STATR & USART_STATR_TXE) == 0u) {
        }
        USART2->DATAR = (uint8_t)*s++;
    }
}

void hw_puthex(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    char b[9];
    uint32_t i;
    for (i = 0; i < 8u; i++) {
        b[i] = hex[(v >> (28u - 4u * i)) & 15u];
    }
    b[8] = 0;
    hw_puts(b);
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
static void fl_unlock(void)
{
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | RCC_HPRE_DIV2;
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    FLASH->MODEKEYR = FLASH_KEY1;             /* fast mode: 256-byte page erase / program */
    FLASH->MODEKEYR = FLASH_KEY2;
}

static int fl_done(uint32_t ctlr_bits)
{
    uint32_t st;
    while (FLASH->STATR & FLASH_STATR_BSY) {
    }
    st = FLASH->STATR;
    FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;   /* write 1 to clear */
    FLASH->CTLR &= ~ctlr_bits;
    FLASH->CTLR |= FLASH_CTLR_LOCK | FLASH_CTLR_FAST_LOCK;
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | RCC_HPRE_DIV1;
    hw_flash_stall = 1;
    return (st & FLASH_STATR_WRPRTERR) ? -1 : 0;
}

int bl_plat_erase_sector(uint32_t addr)
{
    fl_unlock();
    FLASH->CTLR |= FLASH_CTLR_PER;
    FLASH->ADDR = addr;
    FLASH->CTLR |= FLASH_CTLR_STRT;
    return fl_done(FLASH_CTLR_PER);
}

int bl_plat_erase_page(uint32_t addr)
{
    fl_unlock();
    FLASH->CTLR |= FLASH_CTLR_PAGE_ER;
    FLASH->ADDR = addr;
    FLASH->CTLR |= FLASH_CTLR_STRT;
    return fl_done(FLASH_CTLR_PAGE_ER);
}

/* RM 32.5.6: FTPG, 64 word writes (each when WRBSY is clear), PGSTRT, wait for BSY. BENCH: sequence, time */
int bl_plat_program_page(uint32_t addr, const uint8_t *d)
{
    uint32_t i;
    fl_unlock();
    FLASH->CTLR |= FLASH_CTLR_PAGE_PG;
    while (FLASH->STATR & (FLASH_STATR_BSY | STATR_WRBSY)) {
    }
    for (i = 0; i < 64u; i++) {
        const uint8_t *p = d + 4u * i;
        *(volatile uint32_t *)(addr + 4u * i) =
            (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        while (FLASH->STATR & STATR_WRBSY) {
        }
    }
    FLASH->CTLR |= FLASH_CTLR_PG_STRT;
    return fl_done(FLASH_CTLR_PAGE_PG);
}

/* standard half-word programming (descriptor, status records). BENCH: into a page programmed record by record */
int bl_plat_program_half(uint32_t addr, uint16_t v)
{
    fl_unlock();
    FLASH->CTLR |= FLASH_CTLR_PG;
    *(volatile uint16_t *)addr = v;
    return fl_done(FLASH_CTLR_PG);
}
