/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Firmware model and pin-role contract. */
#ifndef RXPOWER_H
#define RXPOWER_H

#include <stdint.h>

#define RXP_LINK_LIMIT_MA  2250u    /* the sender's port limit (PTCs) */
#define RXP_EFF_POINTS     5u

typedef struct {
    uint32_t pin_mw[RXP_EFF_POINTS];        /* input power, rising */
    uint16_t eff_permille[RXP_EFF_POINTS];  /* the module's efficiency there */
} rxp_eff_t;

typedef struct {
    uint32_t vbus_mv, ibus_ma, vb_mv;       /* firmware model */
    uint32_t pin_mw;                        /* V_bus x I_bus */
    uint32_t pout_mw;                       /* estimated output power: pin x efficiency */
    uint16_t eff_permille;                  /* the efficiency used */
    uint16_t link_permille;                 /* I_bus over the link limit, 1000 = at it */
    uint8_t  over_link;                     /* I_bus above the link limit */
} rxp_t;

extern const rxp_eff_t RXP_EFF_DEFAULT;     /* firmware model */

/* the module's efficiency at an input power (clamped to the table's ends) */
uint16_t rxp_eff(const rxp_eff_t *e, uint32_t pin_mw);
void     rxp_update(rxp_t *r, const rxp_eff_t *e, uint32_t vbus_mv, uint32_t ibus_ma, uint32_t vb_mv);

#endif
