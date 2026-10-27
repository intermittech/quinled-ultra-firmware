/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Diff-Power-UR-Digital-M's delay line on the CH32V407, the hardware half: see hw_dline.h. Register usage: the
 * CH32V407 reference manual V1.2: SPI1 (p.340-364, HSRXEN p.364), DMA requests (p.147: SPI1_RX = DMA1 ch2,
 * SPI1_TX = ch3; ch2 / ch3 also carry TIM2_UP, TIM3_CH3 / TIM3_UP, TIM3_CH4, whose DMA enables stay 0), the timers'
 * ITR table (p.244: TIM4 ITR2 = TIM3). Items marked BENCH need silicon. */
#include "ch32v40x.h"
#include "dma_map.h"
#include "hw.h"
#include "hw_priv.h"
#include "hw_dline.h"

uint16_t dl_ring[DL_RING_WORDS + DL_RING_GUARD] __attribute__((aligned(4)));

/* both channels: circular over the ring, 16-bit both sides, priority very high (PL 11; ties to the lower channel,
 * RX; RM p.141) */
#define DL_DMA_CFG (DMA_CFGR1_CIRC | DMA_CFGR1_MINC | DMA_CFGR1_PSIZE_0 | DMA_CFGR1_MSIZE_0 | DMA_CFGR1_PL | DMA_CFGR1_EN)

/* the ring zeroed and SPI1 started (reset, or never enabled; its DMA channels off). MOSI (PB5) is a GPIO LOW until
 * SPE, then SPI1's: it replays the zeroed ring first. Master, full duplex, 16-bit frames (DFF), NSS by software
 * (SSM + SSI, SSOE 0: PA15 stays the ARGB's, RM p.341 / 358-359), BR 000 = PCLK2 100 MHz / 2 = 50 MHz with HSRXEN
 * (HSCR bit 0: "only when divided by 2", RM p.364), mode 0, MSB first (the earliest sample is bit 15). The order: RX
 * channel, TX channel, both DMA enables, then SPE: TXE is set while the SPI is off (STATR reset 0x0002), so the TX
 * channel loads word 0 at once and word 1 when SPE moves it into the shift register: TX runs 2 words ahead of RX.
 * BENCH: SCK continuous at 50 MHz with both DMA channels (no gap between frames; a gap stalls RX and TX together, so
 * the delay holds); the RX - TX distance at 2. */
static void dl_spi_start(void)
{
    uint32_t i;
    pin_cfg(GPIOB, 5, CFG_OUT_PP);
    GPIOB->BCR = 1u << 5;
    for (i = 0; i < DL_RING_WORDS; i++) {
        dl_ring[i] = 0;
    }
    MCU_DMA_SPI_RX->CFGR = 0;
    MCU_DMA_SPI_TX->CFGR = 0;
    SPI1->I2SCFGR = 0;                                   /* I2SMOD 0: SPI */
    SPI1->CTLR1 = SPI_CTLR1_MSTR | SPI_CTLR1_SSM | SPI_CTLR1_SSI | SPI_CTLR1_DFF;
    SPI1->HSCR = SPI_HSCR_HSRXEN;
    DMA1->INTFCR = 0xFFu << 4;                           /* channel 2 / 3 flags cleared */
    MCU_DMA_SPI_RX->PADDR = (uint32_t)&SPI1->DATAR;
    MCU_DMA_SPI_RX->MADDR = (uint32_t)dl_ring;
    MCU_DMA_SPI_RX->CNTR = DL_RING_WORDS;
    MCU_DMA_SPI_TX->PADDR = (uint32_t)&SPI1->DATAR;
    MCU_DMA_SPI_TX->MADDR = (uint32_t)dl_ring;
    MCU_DMA_SPI_TX->CNTR = DL_RING_WORDS;
    MCU_DMA_SPI_RX->CFGR = DL_DMA_CFG;                   /* RX: SPI1 -> ring */
    MCU_DMA_SPI_TX->CFGR = DL_DMA_CFG | DMA_CFGR1_DIR;   /* TX: ring -> SPI1 */
    SPI1->CTLR2 = SPI_CTLR2_RXDMAEN | SPI_CTLR2_TXDMAEN | SPI_CTLR2_ERRIE;
    SPI1->CTLR1 |= SPI_CTLR1_SPE;
    pin_cfg(GPIOB, 5, CFG_AF_PP);                        /* SPI1 MOSI (remap 1) */
}

/* The bit counter: TIM3 CH1 (remap 10: PB4, the pin SPI1 MISO samples) captures falling edges (TI1, filter fCK_INT
 * N = 2: 10 ns at 200 MHz), its TRGO pulses on each capture (MMS 011) and clocks TIM4 (external clock mode 1 on ITR2 =
 * TIM3, RM p.244, 250-258). CC2E-CC4E stay 0 (CH2 is on PB5, the replay). No interrupt, no DMA. */
static void dl_counter_start(void)
{
    TIM3->CTLR1 = 0;
    TIM3->CCER = 0;
    TIM3->PSC = 0;
    TIM3->ATRLR = 0xFFFFu;
    TIM3->CHCTLR1 = (uint16_t)(1u /* CC1S = TI1 */ | (1u << 4) /* IC1F: fCK, N = 2 */);
    TIM3->CHCTLR2 = 0;
    TIM3->CCER = TIM_CC1E | TIM_CC1P;
    TIM3->CTLR2 = (uint16_t)(3u << 4);                   /* MMS = 011 */
    TIM3->DMAINTENR = 0;
    TIM3->CTLR1 = TIM_CEN;
    TIM4->CTLR1 = 0;
    TIM4->CCER = 0;                                      /* CH1 / CH2 pins are PB6 / PB7: never driven */
    TIM4->PSC = 0;
    TIM4->ATRLR = 0xFFFFu;
    TIM4->SMCFGR = (uint16_t)(2u << 4);                  /* TS = ITR2 (TIM3) first: TS only while SMS = 0 */
    TIM4->SMCFGR = (uint16_t)((2u << 4) | 7u);           /* SMS = 111: external clock mode 1 */
    TIM4->DMAINTENR = 0;
    TIM4->CNT = 0;
    TIM4->CTLR1 = TIM_CEN;
}

void dl_hw_start(void)
{
    dl_counter_start();
    dl_spi_start();
    NVIC_SetPriority(SPI1_IRQn, 0x40);                   /* above the tick (0x80; bit 7 preempts) */
    NVIC_EnableIRQ(SPI1_IRQn);
}

void dl_hw_restart(void)
{
    MCU_DMA_SPI_RX->CFGR = 0;
    MCU_DMA_SPI_TX->CFGR = 0;
    RCC->PB2PRSTR |= RCC_SPI1RST;                        /* SPI1 back to reset (OVR, BSY, a half-shifted word: gone) */
    RCC->PB2PRSTR &= ~RCC_SPI1RST;
    dl_spi_start();
}

/* SPI1's interrupt (isr.S): ERRIE is the only source (OVR; MODF can't happen with SSM + SSI, CRC is off). OVR cleared
 * (DATAR, then STATR: RM p.344), ERRIE off, then the app's hook. */
void dl_spi1_isr(void);
void dl_spi1_isr(void)
{
    (void)SPI1->DATAR;
    (void)SPI1->STATR;
    SPI1->CTLR2 = SPI_CTLR2_RXDMAEN | SPI_CTLR2_TXDMAEN;
    dl_isr_fault();
}
