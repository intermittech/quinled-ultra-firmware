/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V407 HSI profile (RM 3.3, p.17-20; DS table 3-13/3-15, p.66). All rates in Hz, durations explicitly labelled.
 * HSI 20 MHz -> PLL x10 = 200 MHz = SYSCLK = HCLK (HPRE /1); PCLK1 = PCLK2 = 100 MHz; TIMxCLK = 2 x PCLK = 200 MHz
 * (the clock tree's x2 rule when the bus prescaler is not 1). Nominal rates: the boot trim below runs the HSI slow
 * until the hub-locked trim (hsitrim) takes over; the measured rate corrects every host-time conversion. */
#ifndef MCU_CLOCK_H
#define MCU_CLOCK_H
#define MCU_HSI_HZ 20000000u
#define MCU_RESET_HSI_HZ 20000000u          /* the clock before SystemInit (bootloader fast path, factory trim) */
#define MCU_PLL_MUL 10u
#define MCU_HCLK_HZ (MCU_HSI_HZ * MCU_PLL_MUL)
#define MCU_PCLK1_HZ (MCU_HCLK_HZ / 2u)
#define MCU_PCLK2_HZ (MCU_HCLK_HZ / 2u)
#define MCU_TIM_HZ MCU_HCLK_HZ
#define MCU_ADC_HZ (MCU_PCLK2_HZ / 8u)
#define MCU_SPI_DIV 2u
#define MCU_POLL_US 50u
#define MCU_POLL_HZ (1000000u / MCU_POLL_US)
#define MCU_TICKS_PER_MS (MCU_POLL_HZ / 1000u)
#define MCU_TICKS_US(us) ((us) / MCU_POLL_US)
#define MCU_POLL_CYC (MCU_HCLK_HZ / MCU_POLL_HZ)
#define MCU_CYC_US(us) ((MCU_HCLK_HZ / 1000000u) * (us))
#define MCU_TIM_10NS_CEIL(n) (((n) * (MCU_TIM_HZ / 1000000u) + 99u) / 100u)
#define MCU_TIM_NS_ROUND(ns) (((ns) * 1ULL * MCU_TIM_HZ + 500000000u) / 1000000000u)
#define MCU_SLEW_DMA_LATE (MCU_CYC_US(4u) / 3u)
#define MCU_CONSOLE_BAUD 115200u
#define MCU_CONSOLE_BRR ((MCU_PCLK2_HZ + MCU_CONSOLE_BAUD / 2u) / MCU_CONSOLE_BAUD)   /* USART1 on PCLK2: 868 */
/* Decided 2026-10-22: before the hub lock the core must stay below 200 MHz. HSI after factory calibration is
 * +-2.2 % over -40..85 C (DS p.66); HSITRIM steps about 40 kHz = 0.2 % (RM p.23, field reset 16). 11 steps below the
 * factory value keep +2.2 % under 200 MHz (199.9); 12 leave one step of margin: 199.5 MHz worst case, -4.55..-0.25 %
 * overall until hsitrim moves the trim to the step at or just below nominal. */
#define MCU_HSI_TRIM_RESET 16u
#define MCU_HSI_BOOT_TRIM_STEPS 12u
#define MCU_HSI_TRIM_PPM_STEP 2000u      /* nominal step, ppm (learned at run time) */
/* the pre-lock sampler/clock range, ppm of nominal, with step-size margin: every window derived from it */
#define MCU_CLOCK_MIN_PPM (-52000)
#define MCU_CLOCK_MAX_PPM 4000
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
