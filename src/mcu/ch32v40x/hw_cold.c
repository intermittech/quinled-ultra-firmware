/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V407 drivers, the cold part: the boot (SystemInit, hw_init: pins, remaps, peripherals, interrupts, the
 * watchdog) and the flash operation (flash_op). Built with -msave-restore like the other cold files. Register usage:
 * the CH32V407 reference manual V1.2 (pages in the comments). */
#include "ch32v40x.h"
#include "dma_map.h"
#include "hw.h"
#include "hw_priv.h"
#include "../../core/flashrec.h"
#include "../../core/lto.h"
#include "../../core/ultra_update.h"
#include "upd_forward.h"

/* The generated binding (bindings.h) supplies portable pad / mode recipes; only this MCU layer encodes registers. */
#define MCU_PORT_A 0u
#define MCU_PORT_B 0x10u
#define MCU_PORT_C 0x20u
#define MCU_PORT_D 0x30u
#define MCU_LEVEL_0 0u
#define MCU_LEVEL_HIGH 0x40u
#define MCU_LEVEL_LOW 0x80u
#define MCU_LEVEL_CLEAR 0xC0u
#define MCU_MODE_OUTPUT CFG_OUT_PP
#define MCU_MODE_OUTPUT_SLOW CFG_OUT_PP_2M
#define MCU_MODE_ALTERNATE CFG_AF_PP
#define MCU_MODE_FLOATING CFG_IN_FLOAT
#define MCU_MODE_PULL CFG_IN_PULL
#define MCU_MODE_ANALOG CFG_ANALOG
#define MCU_PIN(port, pin, level, mode) ((uint16_t)((pin) | MCU_PORT_##port | MCU_LEVEL_##level | (MCU_MODE_##mode << 8)))
static const uint16_t pin_tab[] = { MCU_BOARD_PINS };

/* ---------------------------------------------------------------------------------------------------- clock */
/* HSI 20 MHz (trimmed MCU_HSI_BOOT_TRIM_STEPS low: decided 2026-10-22, clock.h) -> PLL x10 = 200 MHz -> SYSPLLSRC =
 * PLL_CLK, SYSPLLDIV /1 -> SYSCLK = HCLK = 200 MHz; PPRE1 = PPRE2 = /2 (100 MHz); ADCPRE /8. The PLL's source and
 * multiplier only while it is off (RM p.19), SYSPLLSRC only while SYSPLL_GATE = 0 and the gate set before SW
 * selects the PLL (RM p.37). Flash SCKMOD keeps its reset HCLK / 2 = 100 MHz (RM p.582), set before any PLL clock.
 * Called by startup.S. */
void SystemInit(void)
{
    uint32_t trim = MCU_HSI_TRIM_RESET - MCU_HSI_BOOT_TRIM_STEPS;
    RCC->CTLR = (RCC->CTLR & ~RCC_HSITRIM) | RCC_HSION | (trim << RCC_HSITRIM_SHIFT);
    while ((RCC->CTLR & RCC_HSIRDY) == 0u) {
    }
    RCC->CFGR0 = (RCC->CFGR0 & ~(RCC_SW | RCC_HPRE | RCC_PPRE1 | RCC_PPRE2 | RCC_ADCPRE | RCC_PLLSRC | RCC_PLLMUL |
                                 RCC_ADCSRC)) |
                 RCC_PPRE1_DIV2 | RCC_PPRE2_DIV2 | RCC_ADCPRE_DIV8 | RCC_PLLMUL10;
    RCC->CFGR2 = (RCC->CFGR2 & ~(RCC_SYSPLLSRC | RCC_SYSPLL_GATE | RCC_SYSPLLDIV)) | RCC_SYSPLLSRC_PLL;
    RCC->CTLR |= RCC_PLLON;
    while ((RCC->CTLR & RCC_PLLRDY) == 0u) {
    }
    RCC->CFGR2 |= RCC_SYSPLL_GATE;
    RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_PLL;
    while ((RCC->CFGR0 & RCC_SWS) != RCC_SWS_PLL) {
    }
}

/* ------------------------------------------------------------------------------------------------- boot */
static uint32_t reset_flags;

uint32_t hw_reset_flags(void)
{
    return reset_flags;
}

extern void rc_uart_entry(void);                         /* isr.S */

static void rc_hw_init(void)
{
    RCC->PB1PCENR |= RCC_USART7EN;
    RC_USART->BRR = RC_BRR_BASE;                         /* 1 Mbaud (rc_brr per burst) */
    RC_USART->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE;   /* TX only; RE / CLKEN / CTSE / RTSE stay 0 */
    NVIC_SetPriority(RC_IRQ, 0x00);
    PFIC->VTFIDR = (PFIC->VTFIDR & ~(0xFFu << 8)) | ((uint32_t)RC_IRQ << 8);
    PFIC->VTFADDR[1] = (uint32_t)rc_uart_entry | 1u;     /* VTF slot 1 (RM p.95-97) */
    NVIC_EnableIRQ(RC_IRQ);
}

/* the independent watchdog: LSI 25-60 kHz (DS p.66) / 32, reload 500: a reset after 0.27-0.64 s without a kick */
static void watchdog_start(void)
{
    IWDG->CTLR = 0x5555u;
    IWDG->PSCR = 3u;
    IWDG->RLDR = 500u;
    IWDG->CTLR = 0xAAAAu;
    IWDG->CTLR = 0xCCCCu;
}

void hw_init(void)
{
    reset_flags = RCC->RSTSCKR;
    RCC->RSTSCKR |= RCC_RMVF;

    RCC->HBPCENR |= RCC_DMA1EN | RCC_ARGBEN;             /* ARGBEN: HBPCENR bit 7 (RM-CN p.27) */
    RCC->PB2PCENR |= RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_IOPCEN | RCC_IOPDEN | RCC_SPI1EN | RCC_USART1EN;
    RCC->PB1PCENR |= RCC_TIM3EN | RCC_TIM4EN | RCC_PWREN;
    /* the PVD at 2.94 / 2.90 V (PLS 01, RM p.13): flash_op starts nothing below it */
    PWR->CTLR = (PWR->CTLR & ~PWR_CTLR_PLS) | PWR_CTLR_PLS_2V9 | PWR_CTLR_PVDE;

    /* Remaps (RM p.132-140), read-modify-write: SPI1 -> PA15 / PB3 / PB4 / PB5; TIM3 10 -> CH1 PB4 (CH2 on PB5 stays
     * an internal capture channel: CC2E = 0); USART1 11 -> TX PA6; USART7 11 -> TX PA9. TIM2 stays 00 (PA15 free for the
     * ARGB), SW_CFG stays 000 (SWD). */
    AFIO->PCFR1 = (AFIO->PCFR1 & ~(AFIO_PCFR1_TIM3_RM | AFIO_PCFR1_TIM2_RM)) |
                  AFIO_PCFR1_SPI1_RM | AFIO_PCFR1_TIM3_RM_PB4 | AFIO_PCFR1_USART1_RM;
    AFIO->PCFR2 = (AFIO->PCFR2 & ~AFIO_PCFR2_USART7_RM) | AFIO_PCFR2_USART7_RM_PA9 | AFIO_PCFR2_USART1_RM1;

    /* the pins (board.h), in order, each with its level before its mode */
    {
        uint32_t i;
        for (i = 0; i < sizeof pin_tab / sizeof pin_tab[0]; i++) {
            uint32_t t = pin_tab[i], pin = t & 15u, port = (t >> 4) & 3u, lv = t & 0xC0u;
            GPIO_TypeDef *g = port == 0u ? GPIOA : port == 1u ? GPIOB : port == 2u ? GPIOC : GPIOD;
            if (lv == MCU_LEVEL_HIGH) {
                g->BSHR = 1u << pin;
            } else if (lv == MCU_LEVEL_LOW) {
                g->BSHR = 1u << (pin + 16u);
            } else if (lv == MCU_LEVEL_CLEAR) {
                g->BCR = 1u << pin;
            }
            pin_cfg(g, pin, t >> 8);
        }
    }

    /* the delay line's SPI1 error interrupt preempts the tick (bit 7 is the preemption bit: CSR 0x804 preempt = 01) */
    NVIC_SetPriority(SysTicK_IRQn, 0x80);

    /* 20 kHz poll tick: 32-bit SysTick at HCLK, auto-reload at CMP (RM p.80-82) */
    SysTick->CMP = MCU_HCLK_HZ / POLL_HZ - 1u;
    SysTick->SR = 0;
    SysTick->CTLR = (1u << 5) /* INIT */ | (1u << 3) /* STRE */ | (1u << 2) /* HCLK */ | (1u << 1) /* STIE */ | 1u;
    NVIC_EnableIRQ(SysTicK_IRQn);

    /* the console: USART1 115200 8N1 TX only, by its TXE interrupt (hw.c) */
    USART1->BRR = MCU_CONSOLE_BRR;
    USART1->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE;
    NVIC_SetPriority(CONSOLE_IRQN, 0xC0);
    NVIC_EnableIRQ(CONSOLE_IRQN);

    rc_hw_init();
    watchdog_start();
}

/* ---------------------------------------------------------------------------------------------------- flash */
/* flash_op (core/flashrec.h): standard mode (RM 32.5): halfword programming (PG), 4 KB sector erase (PER), the option
 * bytes (OBPG / OBER with OBKEYR). No clock changes: the flash access clock is 100 MHz in every mode (RM p.582). The
 * erase / program commands and their BSY wait run from the zero-wait flash (RM V1.2 p.579, RM-CN p.512: functions
 * related to erase and programming only there; BENCH: execution during BSY); meanwhile only the SysTick enters,
 * through VTF slot 0 to flash_tick (SRAM): the time base and the delay line's SRAM half keep running; mtvec points to
 * flash_fault (SRAM). Every wait is bounded by the operation's
 * deadline in ticks (2 x the declared bound: halfword 2 x 30 ms, sector 2 x 100 ms); a stuck BSY resets the chip. */
#define FLASH_KEY1      0x45670123u
#define FLASH_KEY2      0xCDEF89ABu
extern void flash_tick(void), flash_fault(void);         /* isr.S, .ramtext */
extern volatile uint32_t tick_now;

static __attribute__((noinline)) uint32_t flash_wait(uint32_t bound, uint32_t start)
{
    uint32_t n = 0, sr;
    while ((sr = FLASH->STATR) & FLASH_STATR_BSY) {
        if (tick_now - start >= bound || ++n == MCU_FLASH_BSY_POLLS) {
            NVIC_SystemReset();
            for (;;) {}
        }
    }
    if (tick_now - start >= bound) return FLASH_ERR_TIMEOUT;
    return (sr & FLASH_STATR_WRPRTERR) ? FLASH_ERR_WRP : (sr & FLASH_STATR_EOP) ? FLASH_OK : FLASH_ERR_EOP;
}

/* the delay line's SPI fault, the console and the answer UART: off meanwhile (the SRAM tick keeps the scan) */
#define FLASH_IRQ1 ((1u << (SPI1_IRQn - 32)) | (1u << (CONSOLE_IRQN - 32)))
#define FLASH_IRQ2 (1u << (RC_IRQ - 64))
#define FLASH_TICKS(ms) (UPD_OPERATION_DEADLINE_FACTOR * (ms) * (POLL_HZ / 1000u))

uint32_t flash_op(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr)
{
    uint32_t i, r = FLASH_OK, mtvec, adc = adc_ready, start = tick_now;
    const uint32_t ob = addr == HW_OB_ADDR;
    BARRIER();                                          /* the record (written as its own type) is complete */
    if (PWR->CSR & PWR_CSR_PVDO) {
        return FLASH_ERR_VDD;
    }
    adc_ready = 0;
    __disable_irq();
    if ((TIM2->CHCTLR2 >> 12 & 7u) == GATE_OCM_TOGGLE) {
        gate_mode(GATE_OCM_LO);                         /* the slot plan closes (no re-arm meanwhile: a second frame would pass
                                                           whole); the tick re-arms it in the next quiet LOW after */
    }
    __asm volatile("csrr %0, mtvec" : "=r"(mtvec));
    PFIC->IRER[1] = FLASH_IRQ1;
    PFIC->IRER[2] = FLASH_IRQ2;
    __asm volatile("fence.i" ::: "memory");             /* (QK3: after masking) */
    PFIC->VTFIDR = (PFIC->VTFIDR & ~0xFFu) | SysTicK_IRQn;
    PFIC->VTFADDR[0] = (uint32_t)flash_tick | 1u;
    __asm volatile("csrw mtvec, %0" ::"r"((uint32_t)flash_fault) : "memory");
    watchdog_kick();
    FLASH->KEYR = FLASH_KEY1;                            /* (back to back, interrupts off: RM p.586) */
    FLASH->KEYR = FLASH_KEY2;
    if (ob) {
        FLASH->OBKEYR = FLASH_KEY1;
        FLASH->OBKEYR = FLASH_KEY2;
    }
    __enable_irq();
    if ((FLASH->CTLR & FLASH_CTLR_LOCK) || (ob && !(FLASH->CTLR & FLASH_CTLR_OBWRE))) {
        r = FLASH_ERR_KEY;
    } else {
        if (erase_addr != 0u) {
            FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;   /* (write 1 to clear) */
            FLASH->CTLR |= ob ? FLASH_CTLR_OBER : FLASH_CTLR_PER;
            if (!ob) {
                FLASH->ADDR = erase_addr;
            }
            FLASH->CTLR |= FLASH_CTLR_STRT;
            r = flash_wait(FLASH_TICKS(MCU_UPD_ERASE_MS), start);
            FLASH->CTLR &= ~(FLASH_CTLR_PER | FLASH_CTLR_OBER | FLASH_CTLR_STRT);
            watchdog_kick();
        }
        if (r == FLASH_OK && halfwords != 0u) {
            uint32_t t0 = tick_now;
            FLASH->CTLR |= ob ? FLASH_CTLR_OBPG : FLASH_CTLR_PG;
            for (i = 0; i < halfwords && r == FLASH_OK; i++) {
                FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;
                *(volatile uint16_t *)(addr + 2u * i) = src[i];
                /* a lone halfword (the confirmation) keeps two ticks for readback and cleanup inside its bound */
                r = flash_wait(halfwords == 1u ? FLASH_TICKS(MCU_UPD_PROGRAM_MS) - 2u
                                               : FLASH_TICKS(MCU_UPD_PROGRAM_MS), halfwords == 1u ? start : t0);
                t0 = tick_now;
            }
            FLASH->CTLR &= ~(FLASH_CTLR_PG | FLASH_CTLR_OBPG);
            watchdog_kick();
        }
    }
    FLASH->CTLR = (FLASH->CTLR & ~FLASH_CTLR_OBWRE) | FLASH_CTLR_LOCK;
    __disable_irq();
    PFIC->VTFADDR[0] = 0;
    __asm volatile("csrw mtvec, %0" ::"r"(mtvec) : "memory");
    PFIC->IENR[1] = FLASH_IRQ1;
    PFIC->IENR[2] = FLASH_IRQ2;
    adc_ready = (uint8_t)adc;
    __enable_irq();
    return r;
}
