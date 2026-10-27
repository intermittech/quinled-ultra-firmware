/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V203 HSI profile. All rates in Hz, durations explicitly labelled. */
#ifndef MCU_CLOCK_H
#define MCU_CLOCK_H
#define MCU_HSI_HZ 8000000u
#define MCU_RESET_HSI_HZ 8000000u           /* the clock before SystemInit (bootloader fast path) */
#define MCU_PLL_MUL 18u
#define MCU_HCLK_HZ (MCU_HSI_HZ * MCU_PLL_MUL)
#define MCU_PCLK1_HZ MCU_HCLK_HZ
#define MCU_PCLK2_HZ (MCU_HCLK_HZ / 2u)
#define MCU_TIM_HZ MCU_HCLK_HZ
#define MCU_ADC_HZ (MCU_PCLK2_HZ / 6u)
#define MCU_SPI_DIV 2u
#define MCU_POLL_US 50u
#define MCU_POLL_HZ (1000000u / MCU_POLL_US)
#define MCU_TICKS_PER_MS (MCU_POLL_HZ / 1000u)
#define MCU_TICKS_US(us) ((us) / MCU_POLL_US)
#define MCU_POLL_CYC (MCU_HCLK_HZ / MCU_POLL_HZ)
#define MCU_CYC_US(us) ((MCU_HCLK_HZ / 1000000u) * (us))
/* Small runtime operands remain 32 bit: exactly the established arithmetic. */
#define MCU_TIM_10NS_CEIL(n) (((n) * (MCU_TIM_HZ / 1000000u) + 99u) / 100u)
#define MCU_TIM_NS_ROUND(ns) (((ns) * 1ULL * MCU_TIM_HZ + 500000000u) / 1000000000u)
#define MCU_SLEW_DMA_LATE (MCU_CYC_US(4u) / 3u)
#define MCU_CONSOLE_BAUD 115200u
#define MCU_CONSOLE_BRR (MCU_PCLK1_HZ / MCU_CONSOLE_BAUD)
/* Timer input in physical 10 ns units. Round upward so the requested period
 * is never shortened; reject zero and clamp to the qualified timer range. */
#ifndef __ASSEMBLER__
static inline unsigned int mcu_pwm_units_10ns(unsigned int units)
{
    const unsigned int max_units = 65535u * 100u / (MCU_TIM_HZ / 1000000u);
    return !units ? 0u : units < 1000u ? 1000u : units > max_units ? max_units : units;
}
static inline unsigned int mcu_pwm_period_10ns(unsigned int units)
{
    unsigned int ticks = MCU_TIM_10NS_CEIL(units);
    return !units ? 0u : ticks < MCU_TIM_10NS_CEIL(1000u) ? MCU_TIM_10NS_CEIL(1000u)
         : ticks > 65535u ? 65535u : ticks;
}
#endif
#endif
