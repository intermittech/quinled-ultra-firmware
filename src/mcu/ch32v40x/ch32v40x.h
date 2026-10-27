/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32V407 reference manual V1.2 (EN and CN) and the CH32V407 datasheet V1.3.
 * Only the firmware/bootloader register interface is exposed; offsets are bytes. Page numbers are the
 * reference manual's printed pages (RM) unless noted.
 */
#ifndef QUINLED_CH32V40X_REGISTERS_H
#define QUINLED_CH32V40X_REGISTERS_H

#include <stdint.h>
#include "core_riscv.h"
#include "system_ch32v40x.h"

/* RM 3.4 (p.21-38): reset and clock control */
typedef struct {
    volatile uint32_t CTLR;      /* +0x00 */
    volatile uint32_t CFGR0;     /* +0x04 */
    volatile uint32_t INTR;      /* +0x08 */
    volatile uint32_t PB2PRSTR;  /* +0x0c */
    volatile uint32_t PB1PRSTR;  /* +0x10 */
    volatile uint32_t HBPCENR;   /* +0x14 */
    volatile uint32_t PB2PCENR;  /* +0x18 */
    volatile uint32_t PB1PCENR;  /* +0x1c */
    volatile uint32_t BDCTLR;    /* +0x20 */
    volatile uint32_t RSTSCKR;   /* +0x24 */
    volatile uint32_t HBRSTR;    /* +0x28 */
    volatile uint32_t CFGR2;     /* +0x2c */
    volatile uint32_t CFGR3;     /* +0x30 */
} ql_rcc_regs;

/* RM 10.3 (p.126-131): word-wide port registers */
typedef struct {
    volatile uint32_t CFGLR;     /* +0x00 */
    volatile uint32_t CFGHR;     /* +0x04 */
    volatile uint32_t INDR;      /* +0x08 */
    volatile uint32_t OUTDR;     /* +0x0c */
    volatile uint32_t BSHR;      /* +0x10 */
    volatile uint32_t BCR;       /* +0x14 */
    volatile uint32_t LCKR;      /* +0x18 */
} GPIO_TypeDef;

/* RM 10.3 (p.131-140): group remaps */
typedef struct {
    volatile uint32_t ECR;       /* +0x00 */
    volatile uint32_t PCFR1;     /* +0x04 */
    volatile uint32_t EXTICR[4]; /* +0x08 */
    uint8_t unused_18[0x4];
    volatile uint32_t PCFR2;     /* +0x1c */
} ql_afio_regs;

/* RM 12.3 (p.171-183): the V20x layout at 0x00-0x4c, plus AUX at 0x54 */
typedef struct {
    volatile uint32_t STATR;     /* +0x00 */
    volatile uint32_t CTLR1;     /* +0x04 */
    volatile uint32_t CTLR2;     /* +0x08 */
    volatile uint32_t SAMPTR1;   /* +0x0c */
    volatile uint32_t SAMPTR2;   /* +0x10 */
    uint8_t unused_14[0x18];
    volatile uint32_t RSQR1;     /* +0x2c */
    uint8_t unused_30[0x4];
    volatile uint32_t RSQR3;     /* +0x34 */
    volatile uint32_t ISQR;      /* +0x38 */
    volatile uint32_t IDATAR1;   /* +0x3c */
    volatile uint32_t IDATAR2;   /* +0x40 */
    volatile uint32_t IDATAR3;   /* +0x44 */
    volatile uint32_t IDATAR4;   /* +0x48 */
    volatile uint32_t RDATAR;    /* +0x4c */
} ql_adc_regs;

/* RM 15.4 (p.244-262): 16-bit registers on a 32-bit stride */
typedef struct {
    volatile uint16_t CTLR1; uint8_t unused_02[0x2];      /* +0x00 */
    volatile uint16_t CTLR2; uint8_t unused_06[0x2];      /* +0x04 */
    volatile uint16_t SMCFGR; uint8_t unused_0a[0x2];     /* +0x08 */
    volatile uint16_t DMAINTENR; uint8_t unused_0e[0x2];  /* +0x0c */
    volatile uint16_t INTFR; uint8_t unused_12[0x2];      /* +0x10 */
    volatile uint16_t SWEVGR; uint8_t unused_16[0x2];     /* +0x14 */
    volatile uint16_t CHCTLR1; uint8_t unused_1a[0x2];    /* +0x18 */
    volatile uint16_t CHCTLR2; uint8_t unused_1e[0x2];    /* +0x1c */
    volatile uint16_t CCER; uint8_t unused_22[0x2];       /* +0x20 */
    volatile uint16_t CNT; uint8_t unused_26[0x2];        /* +0x24 */
    volatile uint16_t PSC; uint8_t unused_2a[0x2];        /* +0x28 */
    volatile uint16_t ATRLR; uint8_t unused_2e[0x2];      /* +0x2c */
    uint8_t unused_30[0x4];
    volatile uint16_t CH1CVR; uint8_t unused_36[0x2];     /* +0x34 */
    volatile uint16_t CH2CVR; uint8_t unused_3a[0x2];     /* +0x38 */
    volatile uint16_t CH3CVR; uint8_t unused_3e[0x2];     /* +0x3c */
    volatile uint16_t CH4CVR; uint8_t unused_42[0x2];     /* +0x40 */
} ql_timer_regs;

/* RM 20.4 (p.356-364): 16-bit registers on a 32-bit stride; HSCR at 0x24 */
typedef struct {
    volatile uint16_t CTLR1; uint8_t unused_02[0x2];      /* +0x00 */
    volatile uint16_t CTLR2; uint8_t unused_06[0x2];      /* +0x04 */
    volatile uint16_t STATR; uint8_t unused_0a[0x2];      /* +0x08 */
    volatile uint16_t DATAR; uint8_t unused_0e[0x2];      /* +0x0c */
    uint8_t unused_10[0xc];
    volatile uint16_t I2SCFGR; uint8_t unused_1e[0x2];    /* +0x1c */
    uint8_t unused_20[0x4];
    volatile uint16_t HSCR; uint8_t unused_26[0x2];       /* +0x24 */
} ql_spi_regs;

/* RM 17.10 (p.272-283): listed as 32-bit registers */
typedef struct {
    volatile uint32_t STATR;     /* +0x00 */
    volatile uint32_t DATAR;     /* +0x04 */
    volatile uint32_t BRR;       /* +0x08 */
    volatile uint32_t CTLR1;     /* +0x0c */
    volatile uint32_t CTLR2;     /* +0x10 */
    volatile uint32_t CTLR3;     /* +0x14 */
    volatile uint32_t GPR;       /* +0x18 */
    volatile uint32_t CTLR4;     /* +0x1c */
} ql_usart_regs;

/* RM 11.3 (p.149-157): channels 1-7 on a 0x14 stride from 0x08, channels 8 and up on 0x10 from 0x90 (the register
 * table; the text formula is wrong for x >= 8), their flags in EXTEM_INTFR */
typedef struct {
    volatile uint32_t INTFR;     /* +0x00 */
    volatile uint32_t INTFCR;    /* +0x04 */
} ql_dma_regs;

typedef struct {
    volatile uint32_t CFGR;      /* +0x00 */
    volatile uint32_t CNTR;      /* +0x04 */
    volatile uint32_t PADDR;     /* +0x08 */
    volatile uint32_t MADDR;     /* +0x0c */
} ql_dma_channel;

/* RM 32.4 (p.580-586) */
typedef struct {
    uint8_t unused_00[0x4];
    volatile uint32_t KEYR;      /* +0x04 */
    volatile uint32_t OBKEYR;    /* +0x08 */
    volatile uint32_t STATR;     /* +0x0c */
    volatile uint32_t CTLR;      /* +0x10 */
    volatile uint32_t ADDR;      /* +0x14 */
    uint8_t unused_18[0x4];
    volatile uint32_t OBR;       /* +0x1c */
    volatile uint32_t WPR;       /* +0x20 */
    volatile uint32_t MODEKEYR;  /* +0x24 */
} ql_flash_regs;

typedef struct {
    volatile uint32_t CTLR;      /* +0x00 */
    volatile uint32_t CSR;       /* +0x04 */
} ql_pwr_regs;                   /* RM 2.4 (p.12-14) */

typedef struct {
    volatile uint32_t DATAR;     /* +0x00 */
    volatile uint32_t IDATAR;    /* +0x04 */
    volatile uint32_t CTLR;      /* +0x08 */
} ql_crc_regs;                   /* RM 5.3 (p.44-45) */

typedef struct {
    volatile uint32_t CTLR;      /* +0x00 */
    volatile uint32_t PSCR;      /* +0x04 */
    volatile uint32_t RLDR;      /* +0x08 */
    volatile uint32_t STATR;     /* +0x0c */
} ql_watchdog_regs;              /* RM 7.3 (p.52-55) */

typedef struct {
    volatile uint32_t CTR;       /* +0x00 */
} ql_exten_regs;                 /* RM 33.2 (p.594-595) */

/* RM 28.3 (p.508-513): the addressable-RGB serialiser */
typedef struct {
    volatile uint32_t CTLR;      /* +0x00 */
    volatile uint32_t DAT_ARR;   /* +0x04: bit period, system clock cycles (10 bits) */
    volatile uint32_t RST_ARR;   /* +0x08: reset period (17 bits) */
    volatile uint32_t CCRL;      /* +0x0c: code 0 high time (10 bits) */
    volatile uint32_t CCRH;      /* +0x10: code 1 high time (10 bits) */
    volatile uint32_t DAT_CYC;   /* +0x14: bytes per frame - 1 */
    volatile uint32_t DATAR;     /* +0x18 */
    volatile uint32_t STATR;     /* +0x1c */
    volatile uint32_t CNTR;      /* +0x20 */
} ql_argb_regs;

/* Peripheral addresses: RM chapters 1-33 (facts collected in the port notes) */
#define RCC ((ql_rcc_regs *)(uintptr_t)0x40021000u)
#define GPIOA ((GPIO_TypeDef *)(uintptr_t)0x40010800u)
#define GPIOB ((GPIO_TypeDef *)(uintptr_t)0x40010c00u)
#define GPIOC ((GPIO_TypeDef *)(uintptr_t)0x40011000u)
#define GPIOD ((GPIO_TypeDef *)(uintptr_t)0x40011400u)
#define AFIO ((ql_afio_regs *)(uintptr_t)0x40010000u)
#define ADC1 ((ql_adc_regs *)(uintptr_t)0x40012400u)
#define TIM2 ((ql_timer_regs *)(uintptr_t)0x40000000u)
#define TIM3 ((ql_timer_regs *)(uintptr_t)0x40000400u)
#define TIM4 ((ql_timer_regs *)(uintptr_t)0x40000800u)
#define SPI1 ((ql_spi_regs *)(uintptr_t)0x40013000u)
#define USART1 ((ql_usart_regs *)(uintptr_t)0x40013800u)
#define USART7 ((ql_usart_regs *)(uintptr_t)0x40001c00u)
#define DMA1 ((ql_dma_regs *)(uintptr_t)0x40020000u)
#define DMA1_EXT ((ql_dma_regs *)(uintptr_t)0x400200f0u)   /* EXTEM_INTFR / EXTEM_INTFCR: channels 8-13 */
#define FLASH ((ql_flash_regs *)(uintptr_t)0x40022000u)
#define PWR ((ql_pwr_regs *)(uintptr_t)0x40007000u)
#define CRC ((ql_crc_regs *)(uintptr_t)0x40023000u)
#define IWDG ((ql_watchdog_regs *)(uintptr_t)0x40003000u)
#define EXTEN ((ql_exten_regs *)(uintptr_t)0x40023800u)
#define ARGB ((ql_argb_regs *)(uintptr_t)0x40025000u)
#define DMA1_Channel2 ((ql_dma_channel *)(uintptr_t)0x4002001cu)
#define DMA1_Channel3 ((ql_dma_channel *)(uintptr_t)0x40020030u)
#define DMA1_Channel10 ((ql_dma_channel *)(uintptr_t)0x400200b0u)

/* RCC (RM p.22-38) */
#define RCC_HSION (1u << 0)
#define RCC_HSIRDY (1u << 1)
#define RCC_HSITRIM_SHIFT 3u
#define RCC_HSITRIM (31u << 3)
#define RCC_PLLON (1u << 24)
#define RCC_PLLRDY (1u << 25)
#define RCC_SW (3u << 0)
#define RCC_SW_PLL (2u << 0)
#define RCC_SWS (3u << 2)
#define RCC_SWS_PLL (2u << 2)
#define RCC_HPRE (15u << 4)
#define RCC_PPRE1 (7u << 8)
#define RCC_PPRE1_DIV2 (4u << 8)
#define RCC_PPRE2 (7u << 11)
#define RCC_PPRE2_DIV2 (4u << 11)
#define RCC_ADCPRE (3u << 14)
#define RCC_ADCPRE_DIV8 (3u << 14)
#define RCC_PLLSRC (1u << 16)
#define RCC_PLLMUL (31u << 17)
#define RCC_PLLMUL10 (2u << 17)
#define RCC_ADCSRC (1u << 28)
#define RCC_SYSPLLSRC (7u << 13)            /* CFGR2 */
#define RCC_SYSPLLSRC_PLL (6u << 13)
#define RCC_SYSPLL_GATE (1u << 12)
#define RCC_SYSPLLDIV (7u << 8)
/* HBPCENR: DMA1 0, DMA2 1, CRC 6, ARGB 7 (RM-CN p.27; the EN bit table calls bit 7 reserved) */
#define RCC_DMA1EN (1u << 0)
#define RCC_CRCEN (1u << 6)
#define RCC_ARGBEN (1u << 7)
#define RCC_ARGBRST (1u << 7)               /* HBRSTR */
/* PB2PCENR / PB2PRSTR */
#define RCC_AFIOEN (1u << 0)
#define RCC_IOPAEN (1u << 2)
#define RCC_IOPBEN (1u << 3)
#define RCC_IOPCEN (1u << 4)
#define RCC_IOPDEN (1u << 5)
#define RCC_ADC1EN (1u << 9)
#define RCC_SPI1EN (1u << 12)
#define RCC_SPI1RST (1u << 12)
#define RCC_USART1EN (1u << 14)
/* PB1PCENR */
#define RCC_TIM2EN (1u << 0)
#define RCC_TIM3EN (1u << 1)
#define RCC_TIM4EN (1u << 2)
#define RCC_USART7EN (1u << 7)
#define RCC_PWREN (1u << 28)
/* RSTSCKR (RM p.34-35): the same positions as the V20x */
#define RCC_RMVF (1u << 24)
#define RCC_PINRSTF (1u << 26)
#define RCC_PORRSTF (1u << 27)
#define RCC_SFTRSTF (1u << 28)
#define RCC_IWDGRSTF (1u << 29)
#define RCC_WWDGRSTF (1u << 30)
#define RCC_LPWRRSTF (1u << 31)

/* PWR (RM p.12-14): PLS 01 = 2.94 V rising / 2.90 V falling */
#define PWR_CTLR_PVDE (1u << 4)
#define PWR_CTLR_PLS (3u << 5)
#define PWR_CTLR_PLS_2V9 (1u << 5)
#define PWR_CSR_PVDO (1u << 2)

/* AFIO (RM p.132-140) */
#define AFIO_PCFR1_SPI1_RM (1u << 0)
#define AFIO_PCFR1_USART1_RM (1u << 2)
#define AFIO_PCFR1_TIM2_RM (3u << 8)
#define AFIO_PCFR1_TIM3_RM (3u << 10)
#define AFIO_PCFR1_TIM3_RM_PB4 (2u << 10)  /* CH1 PB4, CH2 PB5 */
#define AFIO_PCFR2_USART7_RM (3u << 22)
#define AFIO_PCFR2_USART7_RM_PA9 (3u << 22)  /* TX PA9, CTS/RTS on unbonded PC8/PC9 */
#define AFIO_PCFR2_USART1_RM1 (1u << 26)

/* DMA (RM p.153-157) */
#define DMA_CFGR1_EN (1u << 0)
#define DMA_CFGR1_TCIE (1u << 1)
#define DMA_CFGR1_TEIE (1u << 3)
#define DMA_CFGR1_DIR (1u << 4)
#define DMA_CFGR1_CIRC (1u << 5)
#define DMA_CFGR1_MINC (1u << 7)
#define DMA_CFGR1_PSIZE_0 (1u << 8)
#define DMA_CFGR1_MSIZE_0 (1u << 10)
#define DMA_CFGR1_PL (3u << 12)
#define DMA_CFGR1_PL_0 (1u << 12)
#define DMA_CFGR1_PL_1 (1u << 13)
#define DMA_TEIF1 (1u << 3)

/* ADC (RM p.172-183) */
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

/* TIM (RM p.247-262) */
#define TIM_CEN (1u << 0)
#define TIM_CC1E (1u << 0)
#define TIM_CC1P (1u << 1)
#define TIM_CC4E (1u << 12)

/* USART (RM p.275-283) */
#define USART_STATR_FE (1u << 1)
#define USART_STATR_NE (1u << 2)
#define USART_STATR_ORE (1u << 3)
#define USART_STATR_TC (1u << 6)
#define USART_STATR_TXE (1u << 7)
#define USART_CTLR1_TE (1u << 3)
#define USART_CTLR1_TCIE (1u << 6)
#define USART_CTLR1_TXEIE (1u << 7)
#define USART_CTLR1_UE (1u << 13)

/* SPI (RM p.357-364) */
#define SPI_CTLR1_MSTR (1u << 2)
#define SPI_CTLR1_SPE (1u << 6)
#define SPI_CTLR1_SSI (1u << 8)
#define SPI_CTLR1_SSM (1u << 9)
#define SPI_CTLR1_RXONLY (1u << 10)
#define SPI_CTLR1_DFF (1u << 11)
#define SPI_CTLR2_RXDMAEN (1u << 0)
#define SPI_CTLR2_TXDMAEN (1u << 1)
#define SPI_CTLR2_ERRIE (1u << 5)
#define SPI_HSCR_HSRXEN (1u << 0)

/* FLASH (RM p.581-586) */
#define FLASH_STATR_BSY (1u << 0)
#define FLASH_STATR_WRBSY (1u << 1)
#define FLASH_STATR_WRPRTERR (1u << 4)
#define FLASH_STATR_EOP (1u << 5)
#define FLASH_CTLR_PG (1u << 0)
#define FLASH_CTLR_PER (1u << 1)
#define FLASH_CTLR_OBPG (1u << 4)
#define FLASH_CTLR_OBER (1u << 5)
#define FLASH_CTLR_STRT (1u << 6)
#define FLASH_CTLR_LOCK (1u << 7)
#define FLASH_CTLR_OBWRE (1u << 9)
#define FLASH_CTLR_FLOCK (1u << 15)
#define FLASH_CTLR_FTPG (1u << 16)
#define FLASH_CTLR_PGSTRT (1u << 21)
#define FLASH_CTLR_EHMOD (1u << 24)
#define FLASH_CTLR_SCKMOD (3u << 25)
#define FLASH_CTLR_SCKMOD_DIV2 (1u << 25)
#define FLASH_OBR_RAM_CODE_MOD (1u << 8)

/* ARGB (RM p.509-513) */
#define ARGB_CTLR_TE (1u << 0)
#define ARGB_CTLR_DMAEN (1u << 1)
#define ARGB_CTLR_MODE (1u << 2)            /* 1: data first, then the reset code */
#define ARGB_STATR_FRHIF (1u << 0)
#define ARGB_STATR_RSTIF (1u << 1)
#define ARGB_STATR_TCIF (1u << 2)
#define ARGB_STATR_STATE (3u << 6)          /* 00 idle, 01 data, 10 reset */

/* RM 9.3 (p.82-85): vector numbers (the number column; the address column is shifted from 26 on) */
enum {
    SysTicK_IRQn = 12,
    SPI1_IRQn = 51,
    USART1_IRQn = 53,
    USART7_IRQn = 88
};

#endif /* QUINLED_CH32V40X_REGISTERS_H */
