/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Ultra-Digital receiver's delay line, the hardware half: see hw_dline.h. Register usage: the CH32FV2x_V3xRM
 * reference manual V2.5 (DMA request map, figure 11-1: SPI1_RX = DMA1 channel 2, SPI1_TX = channel 3; the
 * general-purpose timers' TRGO / ITR table 15-2: TIM4's ITR2 = TIM3). Items marked BENCH need the board. */
#include "ch32v20x.h"
#include "hw.h"
#include "hw_priv.h"
#include "hw_dline.h"

uint16_t dl_ring[DL_RING_WORDS + DL_RING_GUARD] __attribute__((aligned(4)));

/* both channels: circular over the whole ring, 16-bit both sides, priority very high (PL 11; the tie goes to the
 * lower channel, RX) */
#define DL_DMA_CFG (DMA_CFGR1_CIRC | DMA_CFGR1_MINC | DMA_CFGR1_PSIZE_0 | DMA_CFGR1_MSIZE_0 | DMA_CFGR1_PL | DMA_CFGR1_EN)

/* the ring zeroed and the SPI started (SPI1 reset or never enabled, its DMA channels off). MOSI (PA7) is a GPIO LOW
 * until SPE (R215 holds it LOW in between as well), then SPI1's: it replays the zeroed ring first, LOW, for D.
 * Full duplex, master, 16-bit frames (DFF: half the DMA transfers of 8-bit ones; required for the bus load), NSS by
 * software (SSM + SSI: PA4 stays the bus divider), BR 000 = PCLK2 72 MHz / 2 = 36 MHz, mode 0, MSB first (the earliest
 * sample is bit 15, as ws_decode reads it). The order (the brief, design section 3): RX channel, TX channel, both
 * DMA enables, then SPE: TXE is set while the SPI is off, so the TX channel loads word 0 at once and word 1 when SPE
 * moves it into the shift register: TX runs 2 words ahead of RX's write, and word k leaves as word k + 512.
 * BENCH: SCK continuous at 36 MHz full duplex with both DMA channels (no gap between frames; a gap stalls RX and TX
 * together, so the delay holds); the RX - TX distance at 2 (dl_hw_rx_pos / dl_hw_tx_pos). */
static void dl_spi_start(void)
{
    uint32_t i;
    pin_cfg(GPIOA, 7, CFG_OUT_PP);
    GPIOA->BCR = 1u << 7;
    for (i = 0; i < DL_RING_WORDS; i++) {
        dl_ring[i] = 0;
    }
    DMA1_Channel2->CFGR = 0;                             /* (the channels off before their registers: RM; a restart's
                                                            are off already, a first start's never ran) */
    DMA1_Channel3->CFGR = 0;
    SPI1->CTLR1 = SPI_CTLR1_MSTR | SPI_CTLR1_SSM | SPI_CTLR1_SSI | SPI_CTLR1_DFF;
    DMA1->INTFCR = 0xFFu << 4;                           /* channel 2 / 3 flags (GIF, TCIF, HTIF, TEIF): cleared */
    DMA1_Channel2->PADDR = (uint32_t)&SPI1->DATAR;
    DMA1_Channel2->MADDR = (uint32_t)dl_ring;
    DMA1_Channel2->CNTR = DL_RING_WORDS;
    DMA1_Channel3->PADDR = (uint32_t)&SPI1->DATAR;
    DMA1_Channel3->MADDR = (uint32_t)dl_ring;
    DMA1_Channel3->CNTR = DL_RING_WORDS;
    DMA1_Channel2->CFGR = DL_DMA_CFG;                    /* RX: SPI1 -> ring */
    DMA1_Channel3->CFGR = DL_DMA_CFG | DMA_CFGR1_DIR;    /* TX: ring -> SPI1 */
    SPI1->CTLR2 = SPI_CTLR2_RXDMAEN | SPI_CTLR2_TXDMAEN | SPI_CTLR2_ERRIE;   /* ERRIE: OVR -> dl_spi1_isr */
    SPI1->CTLR1 |= SPI_CTLR1_SPE;
    pin_cfg(GPIOA, 7, CFG_AF_PP);                        /* (50 MHz AF push-pull: SPI1_MOSI, no remap) */
}

/* The bit counter: TIM3 CH1 captures PA6's falling edges (TI1, filter fCK_INT N = 2: 14 ns; one per WS28xx bit), its
 * TRGO pulses on each capture (MMS 011: compare pulse, also on a capture) and clocks TIM4 (external clock mode 1 on
 * ITR2 = TIM3): TIM4's count = the falling edges (BOARD 1's DIN edge counter, hw_cold.c). TIM3 counts freely at
 * 144 MHz (its captures need no interrupt, its compares none either). No interrupt from either timer. */
static void dl_counter_start(void)
{
    TIM3->CTLR1 = 0;
    TIM3->PSC = 0;
    TIM3->ATRLR = 0xFFFFu;
    TIM3->CHCTLR1 = (uint16_t)(1u /* CC1S = TI1 */ | (1u << 4) /* IC1F: fCK, N = 2 */);
    TIM3->CHCTLR2 = 0;
    TIM3->CCER = TIM_CC1E | TIM_CC1P;                    /* capture on falling edges */
    TIM3->CTLR2 = (uint16_t)(3u << 4);                   /* MMS = 011 */
    TIM3->DMAINTENR = 0;
    TIM3->CTLR1 = TIM_CEN;
    TIM4->CTLR1 = 0;
    TIM4->PSC = 0;
    TIM4->ATRLR = 0xFFFFu;
    TIM4->SMCFGR = (uint16_t)(2u << 4);                  /* TS = ITR2 (TIM3) first: TS only while SMS = 0 (RM) */
    TIM4->SMCFGR = (uint16_t)((2u << 4) | 7u);           /* SMS = 111: external clock mode 1 */
    TIM4->DMAINTENR = 0;
    TIM4->CNT = 0;
    TIM4->CTLR1 = TIM_CEN;
}

void dl_hw_start(void)
{
    dl_counter_start();
    dl_spi_start();
    NVIC_SetPriority(SPI1_IRQn, 0x40);                   /* above the tick (0x80; nesting: bit 7 preempts) */
    NVIC_EnableIRQ(SPI1_IRQn);
}

void dl_hw_restart(void)
{
    DMA1_Channel2->CFGR = 0;
    DMA1_Channel3->CFGR = 0;
    RCC->APB2PRSTR |= RCC_SPI1RST;                       /* SPI1 back to reset (OVR, BSY, a word half shifted: gone) */
    RCC->APB2PRSTR &= ~RCC_SPI1RST;
    dl_spi_start();
}

/* SPI1's interrupt, entered via isr.S (HPE + mret): ERRIE is the only source (OVR; MODF can't happen with SSM + SSI,
 * CRC is off). OVR cleared (DATAR, then STATR: RM), ERRIE off (no storm if RX stays stopped: a DMA error disables its
 * channel), then the app's hook. */
void dl_spi1_isr(void);
void dl_spi1_isr(void)
{
    (void)SPI1->DATAR;
    (void)SPI1->STATR;
    SPI1->CTLR2 = SPI_CTLR2_RXDMAEN | SPI_CTLR2_TXDMAEN;
    dl_isr_fault();
}
