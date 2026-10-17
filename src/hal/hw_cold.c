/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V203F8U6 drivers, the cold part (review round 9): the boot (SystemInit, hw_init: pins, peripherals, interrupts,
 * the watchdog) and the flash operation (flash_op: ~750 B, which saved 12 registers), built with -msave-restore like
 * the other cold files (tools/build_target.sh: -88 B; moving the boot code too -24 .. -44 B more). The hot helpers,
 * the ISRs, the SRAM trip and what main inlines (hw_adc_init, stack_paint, the option bytes) stay in hw.c: LTO does
 * not inline across a save-restore boundary, and out of line they cost more than -msave-restore saves (pwm_start:
 * +16 .. +44 B, apply_settings inlines it). What the two share: hw_priv.h. */
#include "ch32v20x.h"
#include "hw.h"
#include "hw_priv.h"
#include "../core/flashrec.h"
#include "../core/stagger.h"
#include "../core/lto.h"

/* hw_init's pins (board.h), in order: the pin, its port, an ODR write before the mode, and the CFG nibble (bits 8-11) */
#define PIN_B    0x10u                                   /* GPIOB (else GPIOA) */
#define PIN_HI   0x20u                                   /* ODR 1 first (BSHR set) */
#define PIN_LO   0x40u                                   /* ODR 0 first (BSHR reset) */
#define PIN_CLR  0x80u                                   /* ODR 0 first (BCR) */
#define PIN(port, pin, odr, cfg) ((uint16_t)((pin) | (port) | (odr) | ((cfg) << 8)))
#if BOARD == BOARD_DP_UD
/* the Ultra-Digital receiver (board.h): the four that steer the strip and the line first (direct, the strip line held
 * LOW, receive, MOSI LOW: the bootloader of hardware id 3 set BLANK_N LOW at reset already; core/dline.c releases it at
 * the first quiet gap, with the mux delayed: polls never show on the strip); the answer UART's TX pin (PB10 USART3_TX by
 * default, board.h ANSWER_UART): rc_hw_init; the other candidate (PA9) stays as reset left it */
static const uint16_t pin_tab[] = {
    PIN(0, 10, PIN_CLR, CFG_OUT_PP),                     /* PA10 MUX_SEL: direct */
    PIN(0, 1, PIN_CLR, CFG_OUT_PP),                      /* PA1 BLANK_N: LOW (blank) until dline's BOOT ends */
    PIN(PIN_B, 14, PIN_CLR, CFG_OUT_PP),                 /* PB14 DE + /RE: receive */
    PIN(0, 7, PIN_CLR, CFG_OUT_PP),                      /* PA7 SPI1_MOSI as a GPIO LOW until the SPI runs (hw_dline.c) */
    PIN(0, 6, 0, CFG_IN_FLOAT),                          /* PA6 RS485_RO: SPI1_MISO + TIM3_CH1 */
    PIN(0, 5, 0, CFG_AF_PP),                             /* PA5 SPI1_SCK, internal only (pad not connected) */
    PIN(0, 0, 0, CFG_ANALOG),                            /* PA0 V_B, PA4 the bus, PB0 the bus current */
    PIN(0, 4, 0, CFG_ANALOG),
    PIN(PIN_B, 0, 0, CFG_ANALOG),
    PIN(0, 2, 0, CFG_AF_PP),                             /* PA2 the console's TX (ANSWER_UART 2: the answer's, DI) */
    PIN(0, 3, PIN_HI, CFG_IN_PULL),                      /* PA3 TP6, the test jig's pad: pull-up */
    PIN(PIN_B, 15, PIN_LO, CFG_OUT_PP),                  /* PB15 the status LED, off */
    PIN(PIN_B, 11, 0, CFG_IN_FLOAT),                     /* PB11 USART3_RX on RO: a spare input (RO drives it) */
#if ANSWER_UART != 3
    PIN(PIN_B, 10, PIN_CLR, CFG_IN_PULL),                /* PB10 not the answer's TX (its 0 R open): pull-down */
#endif
    PIN(0, 8, PIN_CLR, CFG_IN_PULL),                     /* PA8 / PB1 spare: pull-down */
    PIN(PIN_B, 1, PIN_CLR, CFG_IN_PULL),
};
#else
static const uint16_t pin_tab[] = {
    PIN(0, 0, 0, CFG_ANALOG),                            /* PA0: the spare input on TP8 (receiver: V_B) */
    PIN(0, 6, 0, CFG_IN_FLOAT),                          /* PA6: DIN */
    PIN(0, 5, 0, CFG_AF_PP),                             /* PA5: SPI1_SCK, internal only (pad not connected) */
    PIN(0, FWD_PIN, PIN_LO, CFG_OUT_PP),                 /* PA1: FWD_EN, the gate closed */
#if ANSWER_UART == 2
    PIN(0, 2, PIN_HI, CFG_IN_PULL),                      /* PA2: the answer's TX (hw.c rc_*): high-Z outside a burst */
#else
    PIN(0, 2, 0, CFG_AF_PP),                             /* PA2 / PA3: the console */
#endif
    PIN(0, 3, PIN_HI, CFG_IN_PULL),                      /* (0.6.7: pull-up, TP6 = the test jig's pad too) */
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
#endif

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

/* ------------------------------------------------------------------------------------------------- boot */
static uint32_t reset_flags;

uint32_t hw_reset_flags(void)
{
    return reset_flags;
}

#if BOARD_RETURN
extern void rc_uart_entry(void);                         /* isr.S */

static void rc_hw_init(void)
{
#if ANSWER_UART == 3
    RCC->APB1PCENR |= 1u << 18;                          /* USART3EN */
#elif ANSWER_UART == 1
    RCC->APB2PCENR |= RCC_USART1EN;                      /* (ANSWER_UART 2: USART2's clock is on in hw_init) */
#endif
    RC_USART->BRR = RC_BRR_BASE;                         /* 1 Mbaud (rc_brr per burst) */
    RC_USART->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE;   /* (as hw.c rc_release: no TXE / TC interrupt) */
#if BOARD == BOARD_DP_UD
    pin_cfg(RC_TX_GPIO, RC_TX_PIN, CFG_AF_PP);           /* (BOARD 3) DI: the USART idles HIGH, DE (PB14) gates it */
#else
    pin_cfg(RC_TX_GPIO, RC_TX_PIN, CFG_IN_PULL);
#endif
    NVIC_SetPriority(RC_IRQ, 0x00);
    PFIC->VTFIDR[1] = RC_IRQ;
    PFIC->VTFADDR[1] = (uint32_t)rc_uart_entry | 1u;
    NVIC_EnableIRQ(RC_IRQ);
}
#endif

/* the independent watchdog (hw.c watchdog_kick): LSI 25-60 kHz / 32, reload 500 -> resets after 0.27-0.64 s (typ.
 * 0.41 s) without a kick. The main loop kicks every pass; its longest stall is a settings write with a page erase
 * (~18 ms). */
static void watchdog_start(void)
{
    IWDG->CTLR = 0x5555u;                                /* unlock PSCR / RLDR */
    IWDG->PSCR = 3u;                                     /* LSI / 32 */
    IWDG->RLDR = 500u;
    IWDG->CTLR = 0xAAAAu;
    IWDG->CTLR = 0xCCCCu;                                /* start (also starts the LSI) */
}

void hw_init(void)
{
    reset_flags = RCC->RSTSCKR;
    RCC->RSTSCKR |= RCC_RMVF;

    RCC->AHBPCENR |= RCC_DMA1EN;
#if BOARD_PWM
    RCC->APB2PCENR |= RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_TIM1EN | RCC_SPI1EN;
    RCC->APB1PCENR |= RCC_TIM2EN | RCC_TIM3EN | (1u << 2) /* TIM4 */ | RCC_USART2EN | RCC_PWREN;
#else
    RCC->APB2PCENR |= RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_SPI1EN;   /* (BOARD 3: no TIM1 / TIM2) */
    RCC->APB1PCENR |= RCC_TIM3EN | (1u << 2) /* TIM4 */ | RCC_USART2EN | RCC_PWREN;
#endif
    /* (0.6.5) the PVD at 2.9 V (as the bootloader): flash_op starts nothing while VDD is below it (DS table 4-17:
     * programming needs >= 2.4 V; a dip write may run while the supply goes away) */
    PWR->CTLR = (PWR->CTLR & ~PWR_CTLR_PLS) | PWR_CTLR_PLS_2V9 | PWR_CTLR_PVDE;

    /* TIM2 partial remap 2: CH3 / CH4 on PB10 / PB11 (CH1 / ETR / CH2 stay on PA0 / PA1, unused by TIM2).
     * Others default: TIM1 CH1 / CH2 = PA8 / PA9, TIM3 CH1 / CH3 / CH4 = PA6 / PB0 / PB1, SPI1 SCK / MISO =
     * PA5 / PA6, USART2 = PA2 / PA3 (BOARD 3: no remap at all: SPI1 PA5-7, TIM3 CH1 PA6, USART3 PB10 / PB11) */
#if BOARD_PWM
    AFIO->PCFR1 = (AFIO->PCFR1 & ~AFIO_PCFR1_TIM2_REMAP) | AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP2;
#endif
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

#if BOARD_DLINE
    /* (BOARD 3) the delay line's SPI1 / DMA / TIM3 / TIM4 and SPI1's error interrupt (above the tick): hw_dline.c
     * dl_hw_start, after hw_init */
    NVIC_SetPriority(SysTicK_IRQn, 0x80);
#else
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
#endif

    /* 20 kHz poll tick */
    SysTick->CMP = 144000000u / POLL_HZ - 1u;            /* HCLK, auto-reload at CMP */
    SysTick->SR = 0;
    SysTick->CTLR = (1u << 5) /* INIT */ | (1u << 3) /* STRE */ | (1u << 2) /* HCLK */ | (1u << 1) /* STIE */ | 1u;
    NVIC_EnableIRQ(SysTicK_IRQn);

#if CONSOLE_TX                                           /* (ANSWER_UART 2: USART2 is the answer UART, rc_hw_init) */
    /* debug UART 115200 8N1, TX by DMA (USART2_TX = DMA1 ch7); the full console's RX too (USART2_RX = DMA1 ch6) */
    USART2->BRR = (uint16_t)(144000000u / 115200u);
#if CONSOLE_FULL
    USART2->CTLR3 = USART_CTLR3_DMAR | (1u << 7) /* DMAT */;
    USART2->CTLR1 = USART_CTLR1_TE | USART_CTLR1_RE | USART_CTLR1_UE;
    DMA1_Channel6->PADDR = (uint32_t)&USART2->DATAR;
    DMA1_Channel6->MADDR = (uint32_t)uart_rx;
    DMA1_Channel6->CNTR = UART_RX_SIZE;
    DMA1_Channel6->CFGR = DMA_CFGR1_CIRC | DMA_CFGR1_MINC | DMA_CFGR1_EN;
#else
    USART2->CTLR3 = 1u << 7 /* DMAT */;
    USART2->CTLR1 = USART_CTLR1_TE | USART_CTLR1_UE;   /* (no RE: PA3 is TP6, the test jig's pad, a GPIO input) */
#endif
    DMA1_Channel7->PADDR = (uint32_t)&USART2->DATAR;
#endif
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
 *    line being typed (uart_getc returns -2 once; the rest of that line, up to CR / LF, is dropped too). ANSWER_UART 2:
 *    USART2 is the answer UART; the same wait covers it, a burst under way is not started under (FLASH_ERR_UART).
 *  - The gate is closed (the tick does no frame work meanwhile): downstream boards miss a frame or two and hold their
 *    last values. A phase slew pauses (no period stays stretched), the injected ADC sequences too. */
#define FLASH_KEY1      0x45670123u
#define FLASH_KEY2      0xCDEF89ABu
#define FLASH_BSY_POLLS 2000000u                         /* ~0.1-0.2 s of BSY polls at 72 MHz: the flash is stuck */
#define FLASH_TC_POLLS  12000u                           /* > shift register + data register at 115200 baud (two bytes:
                                                            174 us) */
#ifndef FLASH_VERIFY
#define FLASH_VERIFY 0                                   /* 1: read each half-word back (BENCH: a read after programming
                                                            may come from the copy made at reset) */
#endif
extern void flash_tick(void), flash_fault(void);         /* isr.S, .ramtext */

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
 * the receiver: also its return channel's USART (board.h ANSWER_UART), on VTF slot 1) */
#if BOARD_DLINE
/* (BOARD 3) the delay line's SPI fault (hw_dline.c) and the reply's USART (the answer UART): off meanwhile; the SRAM tick
 * (main_ud.c flash_poll_isr -> dl_ram_tick) keeps the scan, the zeroing and the RX - TX check going, the DMA the ring */
#define FLASH_IRQ1 ((1u << (SPI1_IRQn - 32)) | (1u << (RC_IRQ - 32)))
#elif BOARD_RETURN
#define FLASH_IRQ1 ((1u << (TIM2_IRQn - 32)) | (1u << (TIM3_IRQn - 32)) | (1u << (TIM4_IRQn - 32)) | \
                    (1u << (RC_IRQ - 32)))
#else
#define FLASH_IRQ1 ((1u << (TIM2_IRQn - 32)) | (1u << (TIM3_IRQn - 32)) | (1u << (TIM4_IRQn - 32)))
#endif

uint32_t flash_op(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr)
{
    uint32_t i, r = FLASH_OK, cfgr, mtvec, adc = adc_ready, txctl;
    const uint32_t ob = addr == HW_OB_ADDR;              /* (0.6.8) the option bytes: OBKEYR, OPTER / OPTPG (RM 32.6) */
#if CONSOLE_FULL
    uint32_t rxw;
#endif
#if BOARD_PWM
    uint16_t slewing;
#endif
    BARRIER();                                          /* the record (written as its own type) is complete */
    if (PWR->CSR & PWR_CSR_PVDO) {
        return FLASH_ERR_VDD;                            /* (0.6.5) VDD below 2.9 V: the write stays pending */
    }
    adc_ready = 0;                                       /* no new injected sequence (hw_adc_ms) */
    /* (review round 6, gpt-fw R6-05) the console's TX: its DMA requests paused (the channel keeps its address and
     * count), the bytes already in the UART finish (two at most: 174 us), then the clock halves; the line goes on
     * after the operation at the restored baud. Never a byte at a baud changing under it, and no write ever waits for
     * a line (a dip write neither). TC not set by then: not started (FLASH_ERR_UART). */
    txctl = USART2->CTLR3;
    USART2->CTLR3 = txctl & ~USART_CTLR3_DMAT;
    for (i = 0; i < FLASH_TC_POLLS && !(USART2->STATR & USART_STATR_TC); i++) {
    }
    if (!(USART2->STATR & USART_STATR_TC)) {
        USART2->CTLR3 = txctl;
        adc_ready = (uint8_t)adc;
        return FLASH_ERR_UART;
    }
    __disable_irq();
#if BOARD_PWM                                            /* (BOARD 3: no gate, no PWM; its DMA ring runs on) */
    gate_force_closed();
    slewing = (uint16_t)((TIM2->DMAINTENR | TIM3->DMAINTENR) & TIM_UIE);
    TIM2->DMAINTENR &= (uint16_t)~TIM_UIE;
    TIM3->DMAINTENR &= (uint16_t)~TIM_UIE;
    if (!cur_spread) {                                   /* spread spectrum: the DMA sets every period's length */
        TIM2->ATRLR = (uint16_t)(cur_steps - 1u);
        TIM3->ATRLR = (uint16_t)(cur_steps - 1u);
    }
    slew_pend2 = slew_pend3 = 0;
#endif
#if CONSOLE_FULL
    rxw = uart_rx_w();
#endif
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
    if (ob) {
        FLASH->OBKEYR = FLASH_KEY1;                      /* the option bytes' own lock (OBWRE) */
        FLASH->OBKEYR = FLASH_KEY2;
    }
    __enable_irq();
    if ((FLASH->CTLR & (FLASH_CTLR_LOCK | FLASH_CTLR_FAST_LOCK)) || (ob && !(FLASH->CTLR & FLASH_CTLR_OPTWRE))) {
        r = FLASH_ERR_KEY;
    } else {
        if (erase_addr != 0u) {
            FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;   /* (write 1 to clear) */
            FLASH->CTLR |= ob ? FLASH_CTLR_OPTER : FLASH_CTLR_PAGE_ER;   /* (the option bytes: all of them) */
            FLASH->ADDR = erase_addr;
            FLASH->CTLR |= FLASH_CTLR_STRT;
            r = flash_wait();
            FLASH->CTLR &= ~(FLASH_CTLR_PAGE_ER | FLASH_CTLR_OPTER | FLASH_CTLR_STRT);
            watchdog_kick();
        }
        if (r == FLASH_OK && halfwords != 0u) {
            FLASH->CTLR |= ob ? FLASH_CTLR_OPTPG : FLASH_CTLR_PG;
            for (i = 0; i < halfwords && r == FLASH_OK; i++) {
                FLASH->STATR = FLASH_STATR_EOP | FLASH_STATR_WRPRTERR;
                *(volatile uint16_t *)(addr + 2u * i) = src[i];
                r = flash_wait();
#if FLASH_VERIFY
                if (r == FLASH_OK && (*(volatile const uint16_t *)(addr + 2u * i) & (ob ? 0xFFu : 0xFFFFu)) != src[i]) {
                    r = FLASH_ERR_VERIFY;
                }
#endif
            }
            FLASH->CTLR &= ~(FLASH_CTLR_PG | FLASH_CTLR_OPTPG);
            watchdog_kick();
        }
    }
    FLASH->CTLR = (FLASH->CTLR & ~FLASH_CTLR_OPTWRE) | FLASH_CTLR_LOCK | FLASH_CTLR_FAST_LOCK;   /* (OBWRE: 0 locks) */
    __disable_irq();
    flash_clock(cfgr, 0);
    PFIC->VTFADDR[0] = 0;
    __asm volatile("csrw mtvec, %0" ::"r"(mtvec) : "memory");
    PFIC->IENR[1] = FLASH_IRQ1;
#if CONSOLE_FULL
    if (uart_rx_w() != rxw || (USART2->STATR & (USART_STATR_ORE | USART_STATR_FE | USART_STATR_NE))) {
        (void)USART2->DATAR;                             /* (STATR then DATAR: the error flags clear) */
        uart_rx_r = uart_rx_w();                         /* received at the halved baud: dropped, the line too */
        uart_rx_drop = 1;
    }
#endif
    adc_ready = (uint8_t)adc;
    USART2->CTLR3 = txctl;                               /* (r6) the line goes on at the restored baud */
    __enable_irq();
#if BOARD_PWM
    if (slewing) {
        TIM2->INTFR = (uint16_t)~TIM_UIF;
        TIM3->INTFR = (uint16_t)~TIM_UIF;
        TIM2->DMAINTENR |= TIM_UIE;
        TIM3->DMAINTENR |= TIM_UIE;
    }
#endif
    return r;
}
