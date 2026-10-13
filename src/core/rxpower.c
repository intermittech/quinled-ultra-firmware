/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Diff-Power ultra receiver, power from the bus side: see rxpower.h. */
#include "rxpower.h"

const rxp_eff_t RXP_EFF_DEFAULT = {
    { 5000u, 15000u, 30000u, 60000u, 100000u },
    { 800u, 880u, 920u, 930u, 920u },
};

uint16_t rxp_eff(const rxp_eff_t *e, uint32_t pin_mw)
{
    uint32_t i;
    if (pin_mw <= e->pin_mw[0]) {
        return e->eff_permille[0];
    }
    for (i = 1; i < RXP_EFF_POINTS; i++) {
        if (pin_mw <= e->pin_mw[i]) {
            uint32_t x0 = e->pin_mw[i - 1u], x1 = e->pin_mw[i];
            int32_t y0 = e->eff_permille[i - 1u], y1 = e->eff_permille[i];
            return (uint16_t)(y0 + (y1 - y0) * (int32_t)((pin_mw - x0) / 100u) / (int32_t)((x1 - x0) / 100u));
        }
    }
    return e->eff_permille[RXP_EFF_POINTS - 1u];
}

void rxp_update(rxp_t *r, const rxp_eff_t *e, uint32_t vbus_mv, uint32_t ibus_ma, uint32_t vb_mv)
{
    r->vbus_mv = vbus_mv;
    r->ibus_ma = ibus_ma;
    r->vb_mv = vb_mv;
    r->pin_mw = vbus_mv / 10u * ibus_ma / 100u;                   /* <= 6553 x 65535 / 100 */
    r->eff_permille = rxp_eff(e, r->pin_mw);
    r->pout_mw = r->pin_mw / 10u * r->eff_permille / 100u;
    r->link_permille = (uint16_t)(ibus_ma * 1000u / RXP_LINK_LIMIT_MA > 65535u ? 65535u
                                                                                : ibus_ma * 1000u / RXP_LINK_LIMIT_MA);
    r->over_link = (uint8_t)(ibus_ma > RXP_LINK_LIMIT_MA);
}
