/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32FV2x/V3x reference manual V2.5 and CH32V203
 * datasheet V3.0. Only the firmware/bootloader register interface is exposed.
 * Offsets are bytes. Reserved storage has no firmware-visible register name.
 */
#ifndef QUINLED_CH32V20X_REGISTERS_H
#define QUINLED_CH32V20X_REGISTERS_H

#include <stdint.h>
#include "core_riscv.h"
#include "system_ch32v20x.h"

typedef struct {
    volatile uint32_t CTLR; /* +0x00 */
    volatile uint32_t CFGR0; /* +0x04 */
    uint8_t unused_08[0x4];
    volatile uint32_t APB2PRSTR; /* +0x0c */
    uint8_t unused_10[0x4];
    volatile uint32_t AHBPCENR; /* +0x14 */
    volatile uint32_t APB2PCENR; /* +0x18 */
    volatile uint32_t APB1PCENR; /* +0x1c */
    uint8_t unused_20[0x4];
    volatile uint32_t RSTSCKR; /* +0x24 */
} ql_rcc_regs;

typedef struct {
    volatile uint32_t CFGLR; /* +0x00 */
    volatile uint32_t CFGHR; /* +0x04 */
    volatile uint32_t INDR; /* +0x08 */
    volatile uint32_t OUTDR; /* +0x0c */
    volatile uint32_t BSHR; /* +0x10 */
    volatile uint32_t BCR; /* +0x14 */
} GPIO_TypeDef;

typedef struct {
    uint8_t unused_00[0x4];
    volatile uint32_t PCFR1; /* +0x04 */
} ql_afio_regs;

typedef struct {
    volatile uint32_t STATR; /* +0x00 */
    volatile uint32_t CTLR1; /* +0x04 */
    volatile uint32_t CTLR2; /* +0x08 */
    volatile uint32_t SAMPTR1; /* +0x0c */
    volatile uint32_t SAMPTR2; /* +0x10 */
    uint8_t unused_14[0x18];
    volatile uint32_t RSQR1; /* +0x2c */
    uint8_t unused_30[0x4];
    volatile uint32_t RSQR3; /* +0x34 */
    volatile uint32_t ISQR; /* +0x38 */
    volatile uint32_t IDATAR1; /* +0x3c */
    volatile uint32_t IDATAR2; /* +0x40 */
    volatile uint32_t IDATAR3; /* +0x44 */
    volatile uint32_t IDATAR4; /* +0x48 */
    volatile uint32_t RDATAR; /* +0x4c */
} ql_adc_regs;

typedef struct {
    volatile uint16_t CTLR1; /* +0x00 */
    uint8_t unused_02[0x2];
    volatile uint16_t CTLR2; /* +0x04 */
    uint8_t unused_06[0x2];
    volatile uint16_t SMCFGR; /* +0x08 */
    uint8_t unused_0a[0x2];
    volatile uint16_t DMAINTENR; /* +0x0c */
    uint8_t unused_0e[0x2];
    volatile uint16_t INTFR; /* +0x10 */
    uint8_t unused_12[0x2];
    volatile uint16_t SWEVGR; /* +0x14 */
    uint8_t unused_16[0x2];
    volatile uint16_t CHCTLR1; /* +0x18 */
    uint8_t unused_1a[0x2];
    volatile uint16_t CHCTLR2; /* +0x1c */
    uint8_t unused_1e[0x2];
    volatile uint16_t CCER; /* +0x20 */
    uint8_t unused_22[0x2];
    volatile uint16_t CNT; /* +0x24 */
    uint8_t unused_26[0x2];
    volatile uint16_t PSC; /* +0x28 */
    uint8_t unused_2a[0x2];
    volatile uint16_t ATRLR; /* +0x2c */
    uint8_t unused_2e[0x2];
    volatile uint16_t RPTCR; /* +0x30 */
    uint8_t unused_32[0x2];
    volatile uint16_t CH1CVR; /* +0x34 */
    uint8_t unused_36[0x2];
    volatile uint16_t CH2CVR; /* +0x38 */
    uint8_t unused_3a[0x2];
    volatile uint16_t CH3CVR; /* +0x3c */
    uint8_t unused_3e[0x2];
    volatile uint16_t CH4CVR; /* +0x40 */
    uint8_t unused_42[0x2];
    volatile uint16_t BDTR; /* +0x44 */
    uint8_t unused_46[0x2];
    volatile uint16_t DMACFGR; /* +0x48 */
    uint8_t unused_4a[0x2];
    volatile uint16_t DMAADR; /* +0x4c */
} ql_timer_regs;

typedef struct {
    volatile uint16_t CTLR1; /* +0x00 */
    uint8_t unused_02[0x2];
    volatile uint16_t CTLR2; /* +0x04 */
    uint8_t unused_06[0x2];
    volatile uint16_t STATR; /* +0x08 */
    uint8_t unused_0a[0x2];
    volatile uint16_t DATAR; /* +0x0c */
} ql_spi_regs;

typedef struct {
    volatile uint16_t STATR; /* +0x00 */
    uint8_t unused_02[0x2];
    volatile uint16_t DATAR; /* +0x04 */
    uint8_t unused_06[0x2];
    volatile uint16_t BRR; /* +0x08 */
    uint8_t unused_0a[0x2];
    volatile uint16_t CTLR1; /* +0x0c */
    uint8_t unused_0e[0x6];
    volatile uint16_t CTLR3; /* +0x14 */
} ql_usart_regs;

typedef struct {
    volatile uint32_t INTFR; /* +0x00 */
    volatile uint32_t INTFCR; /* +0x04 */
} ql_dma_regs;

typedef struct {
    volatile uint32_t CFGR; /* +0x00 */
    volatile uint32_t CNTR; /* +0x04 */
    volatile uint32_t PADDR; /* +0x08 */
    volatile uint32_t MADDR; /* +0x0c */
} ql_dma_channel;

typedef struct {
    uint8_t unused_00[0x4];
    volatile uint32_t KEYR; /* +0x04 */
    volatile uint32_t OBKEYR; /* +0x08 */
    volatile uint32_t STATR; /* +0x0c */
    volatile uint32_t CTLR; /* +0x10 */
    volatile uint32_t ADDR; /* +0x14 */
    uint8_t unused_18[0x8];
    volatile uint32_t WPR; /* +0x20 */
    volatile uint32_t MODEKEYR; /* +0x24 */
} ql_flash_regs;

typedef struct {
    volatile uint32_t CTLR; /* +0x00 */
    volatile uint32_t CSR; /* +0x04 */
} ql_pwr_regs;

typedef struct {
    volatile uint32_t DATAR; /* +0x00 */
    uint8_t unused_04[0x4];
    volatile uint32_t CTLR; /* +0x08 */
} ql_crc_regs;

typedef struct {
    volatile uint32_t CTLR; /* +0x00 */
    volatile uint32_t PSCR; /* +0x04 */
    volatile uint32_t RLDR; /* +0x08 */
} ql_watchdog_regs;

typedef struct {
    volatile uint32_t EXTEN_CTR; /* +0x00 */
} ql_exten_regs;

/* RM chapters 2, 3, 5, 8, 10-15, 18, 20, 32 and 33: peripheral addresses. */
#define RCC ((ql_rcc_regs *)(uintptr_t)0x40021000u)
#define GPIOA ((GPIO_TypeDef *)(uintptr_t)0x40010800u)
#define GPIOB ((GPIO_TypeDef *)(uintptr_t)0x40010c00u)
#define AFIO ((ql_afio_regs *)(uintptr_t)0x40010000u)
#define ADC1 ((ql_adc_regs *)(uintptr_t)0x40012400u)
#define TIM1 ((ql_timer_regs *)(uintptr_t)0x40012c00u)
#define TIM2 ((ql_timer_regs *)(uintptr_t)0x40000000u)
#define TIM3 ((ql_timer_regs *)(uintptr_t)0x40000400u)
#define TIM4 ((ql_timer_regs *)(uintptr_t)0x40000800u)
#define SPI1 ((ql_spi_regs *)(uintptr_t)0x40013000u)
#define USART1 ((ql_usart_regs *)(uintptr_t)0x40013800u)
#define USART2 ((ql_usart_regs *)(uintptr_t)0x40004400u)
#define DMA1 ((ql_dma_regs *)(uintptr_t)0x40020000u)
#define FLASH ((ql_flash_regs *)(uintptr_t)0x40022000u)
#define PWR ((ql_pwr_regs *)(uintptr_t)0x40007000u)
#define CRC ((ql_crc_regs *)(uintptr_t)0x40023000u)
#define IWDG ((ql_watchdog_regs *)(uintptr_t)0x40003000u)
#define EXTEN ((ql_exten_regs *)(uintptr_t)0x40023800u)
#define DMA1_Channel1 ((ql_dma_channel *)(uintptr_t)0x40020008u)
#define DMA1_Channel2 ((ql_dma_channel *)(uintptr_t)0x4002001cu)
#define DMA1_Channel3 ((ql_dma_channel *)(uintptr_t)0x40020030u)
#define DMA1_Channel4 ((ql_dma_channel *)(uintptr_t)0x40020044u)
#define DMA1_Channel5 ((ql_dma_channel *)(uintptr_t)0x40020058u)
#define DMA1_Channel6 ((ql_dma_channel *)(uintptr_t)0x4002006cu)
#define DMA1_Channel7 ((ql_dma_channel *)(uintptr_t)0x40020080u)

/* RM 3.4: clock selection, prescalers, enables and reset flags. */
#define RCC_HSION (1u << 0)
#define RCC_PLLON (1u << 24)
#define RCC_PLLRDY (1u << 25)
#define RCC_SW (3u << 0)
#define RCC_SW_PLL (2u << 0)
#define RCC_SWS (3u << 2)
#define RCC_HPRE (15u << 4)
#define RCC_HPRE_DIV1 (0u << 4)
#define RCC_HPRE_DIV2 (8u << 4)
#define RCC_PPRE1 (7u << 8)
#define RCC_PPRE2 (7u << 11)
#define RCC_PPRE2_DIV2 (4u << 11)
#define RCC_ADCPRE (3u << 14)
#define RCC_ADCPRE_DIV6 (2u << 14)
#define RCC_PLLSRC (1u << 16)
#define RCC_PLLSRC_HSI_Div2 (0u << 16)
#define RCC_PLLXTPRE (1u << 17)
#define RCC_PLLMULL (15u << 18)
#define RCC_PLLMULL18 (15u << 18)
#define RCC_DMA1EN (1u << 0)
#define RCC_CRCEN (1u << 6)
#define RCC_AFIOEN (1u << 0)
#define RCC_IOPAEN (1u << 2)
#define RCC_IOPBEN (1u << 3)
#define RCC_ADC1EN (1u << 9)
#define RCC_TIM1EN (1u << 11)
#define RCC_SPI1EN (1u << 12)
#define RCC_SPI1RST (1u << 12)
#define RCC_USART1EN (1u << 14)
#define RCC_TIM2EN (1u << 0)
#define RCC_TIM3EN (1u << 1)
#define RCC_USART2EN (1u << 17)
#define RCC_PWREN (1u << 28)
#define RCC_RMVF (1u << 24)
#define RCC_PINRSTF (1u << 26)
#define RCC_PORRSTF (1u << 27)
#define RCC_SFTRSTF (1u << 28)
#define RCC_IWDGRSTF (1u << 29)
#define RCC_WWDGRSTF (1u << 30)
#define RCC_LPWRRSTF (1u << 31)

/* RM 2.4 and 33.3: voltage detector and PLL input. */
#define PWR_CTLR_PVDE (1u << 4)
#define PWR_CTLR_PLS (7u << 5)
/* Retain the firmware's level-7 selection; the threshold depends on VLEVEL. */
#define PWR_CTLR_PLS_2V9 (7u << 5)
#define PWR_CSR_PVDO (1u << 2)
#define EXTEN_PLL_HSI_PRE (1u << 4)

/* RM 10.4: timer pin remapping. */
#define AFIO_PCFR1_TIM2_REMAP (3u << 8)
#define AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP2 (2u << 8)

/* RM 11.4: DMA channel configuration and transfer-error status. */
#define DMA_CFGR1_EN (1u << 0)
#define DMA_CFGR1_DIR (1u << 4)
#define DMA_CFGR1_CIRC (1u << 5)
#define DMA_CFGR1_MINC (1u << 7)
#define DMA_CFGR1_PSIZE_0 (1u << 8)
#define DMA_CFGR1_PSIZE_1 (1u << 9)
#define DMA_CFGR1_MSIZE_0 (1u << 10)
#define DMA_CFGR1_MSIZE_1 (1u << 11)
#define DMA_CFGR1_PL (3u << 12)
#define DMA_CFGR1_PL_0 (1u << 12)
#define DMA_CFGR1_PL_1 (1u << 13)
#define DMA_TEIF1 (1u << 3)

/* RM 12.3: ADC status and conversion control. */
#define ADC_JEOC (1u << 2)
#define ADC_STRT (1u << 4)
#define ADC_SCAN (1u << 8)
#define ADC_ADON (1u << 0)
#define ADC_CONT (1u << 1)
#define ADC_CAL (1u << 2)
#define ADC_RSTCAL (1u << 3)
#define ADC_JEXTSEL (7u << 12)
#define ADC_JEXTTRIG (1u << 15)
#define ADC_JSWSTART (1u << 21)
#define ADC_TSVREFE (1u << 23)

/* RM 14.4 and 15.4: timer operation, DMA, compare and output control. */
#define TIM_CEN (1u << 0)
#define TIM_ARPE (1u << 7)
#define TIM_UIE (1u << 0)
#define TIM_UDE (1u << 8)
#define TIM_UIF (1u << 0)
#define TIM_CC2IF (1u << 2)
#define TIM_UG (1u << 0)
#define TIM_OC1PE (1u << 3)
#define TIM_OC2PE (1u << 11)
#define TIM_OC3PE (1u << 3)
#define TIM_OC4PE (1u << 11)
#define TIM_CC1E (1u << 0)
#define TIM_CC1P (1u << 1)
#define TIM_CC2E (1u << 4)
#define TIM_CC2NE (1u << 6)
#define TIM_CC2NP (1u << 7)
#define TIM_CC3E (1u << 8)
#define TIM_CC4E (1u << 12)
#define TIM_MOE (1u << 15)

/* RM 18.10: serial status, transmitter/receiver and DMA control. */
#define USART_STATR_FE (1u << 1)
#define USART_STATR_NE (1u << 2)
#define USART_STATR_ORE (1u << 3)
#define USART_STATR_TC (1u << 6)
#define USART_STATR_TXE (1u << 7)
#define USART_CTLR1_RE (1u << 2)
#define USART_CTLR1_TE (1u << 3)
#define USART_CTLR1_TCIE (1u << 6)
#define USART_CTLR1_TXEIE (1u << 7)
#define USART_CTLR1_UE (1u << 13)
#define USART_CTLR3_DMAR (1u << 6)
#define USART_CTLR3_DMAT (1u << 7)

/* RM 20.4: SPI master, word size, receive mode and DMA control. */
#define SPI_CTLR1_MSTR (1u << 2)
#define SPI_CTLR1_SPE (1u << 6)
#define SPI_CTLR1_SSI (1u << 8)
#define SPI_CTLR1_SSM (1u << 9)
#define SPI_CTLR1_RXONLY (1u << 10)
#define SPI_CTLR1_DFF (1u << 11)
#define SPI_CTLR2_RXDMAEN (1u << 0)
#define SPI_CTLR2_TXDMAEN (1u << 1)
#define SPI_CTLR2_ERRIE (1u << 5)

/* RM 32.7: flash status, locking and program/erase commands. */
#define FLASH_STATR_BSY (1u << 0)
#define FLASH_STATR_WRPRTERR (1u << 4)
#define FLASH_STATR_EOP (1u << 5)
#define FLASH_CTLR_PG (1u << 0)
#define FLASH_CTLR_PER (1u << 1)
#define FLASH_CTLR_OPTPG (1u << 4)
#define FLASH_CTLR_OPTER (1u << 5)
#define FLASH_CTLR_STRT (1u << 6)
#define FLASH_CTLR_LOCK (1u << 7)
#define FLASH_CTLR_OPTWRE (1u << 9)
#define FLASH_CTLR_FAST_LOCK (1u << 15)
#define FLASH_CTLR_PAGE_PG (1u << 16)
#define FLASH_CTLR_PAGE_ER (1u << 17)
#define FLASH_CTLR_PG_STRT (1u << 21)

/* RM 9.3, D6 interrupt numbers (including the core exception entries). */
enum {
    SysTicK_IRQn = 12,
    TIM2_IRQn = 44,
    TIM3_IRQn = 45,
    TIM4_IRQn = 46,
    SPI1_IRQn = 51,
    USART1_IRQn = 53
};

#endif /* QUINLED_CH32V20X_REGISTERS_H */
