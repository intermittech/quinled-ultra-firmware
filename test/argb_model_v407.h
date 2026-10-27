/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* A host model of the V407 ARGB serialiser for argbenc.c (WS_ARGB_HW, test/test_udfx_v407.c): a frame of n bytes at the
 * bit period `per` HCLK cycles takes (lead + 8 n per) / 64 ring words; the DMA's read position runs linearly. */
#ifndef ARGB_MODEL_V407_H
#define ARGB_MODEL_V407_H
#include <stdint.h>
#if defined(_MSC_VER)
#define __attribute__(x)
#endif
#define ARGB_LEAD_CYC 200u
#define ARGB_ST_RUN  0u
#define ARGB_ST_OK   1u
#define ARGB_ST_FAIL 2u
extern uint32_t m_now;                  /* the tick's ring word */
extern uint32_t m_running, m_start, m_n, m_per, m_fail;
void m_frame_start(const uint8_t *b, uint32_t n, uint32_t per);
static inline uint32_t argb_hw_now(uint32_t w) { (void)w; return m_now + 1u; }
static inline void argb_hw_start(const uint8_t *b, uint32_t n, uint32_t per, uint32_t t0, uint32_t t1)
{
    (void)t0; (void)t1;
    m_frame_start(b, n, per);
}
static inline uint32_t m_dur(void) { return (ARGB_LEAD_CYC + m_n * 8u * m_per + 63u) / 64u; }
static inline uint32_t argb_hw_left(void)
{
    const uint32_t el = m_now - m_start, d = m_dur();
    return el >= d ? 0u : m_n - (uint32_t)((uint64_t)m_n * el / d);
}
static inline uint32_t argb_hw_status(void)
{
    if (m_fail) return ARGB_ST_FAIL;
    return m_now - m_start >= m_dur() ? ARGB_ST_OK : ARGB_ST_RUN;
}
static inline void argb_hw_stop(void) { m_running = 0; }
#endif
