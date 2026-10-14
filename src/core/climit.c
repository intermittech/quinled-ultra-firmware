/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "climit.h"

void climit_eval(climit_t *c, uint16_t user_ma, uint16_t board_ma, uint16_t fuse_ma, uint32_t connected_ma)
{
    uint32_t l = 0xFFFFu;
    uint8_t src = CLIMIT_SRC_NONE;
    if (board_ma != 0u) {
        l = board_ma;
        src = CLIMIT_SRC_BOARD;
    }
    if (user_ma != 0u && user_ma < l) {
        l = user_ma;
        src = CLIMIT_SRC_USER;
    }
    if (fuse_ma != 0u && fuse_ma < l) {
        l = fuse_ma;
        src = CLIMIT_SRC_FUSE;
    }
    c->limit_ma = (uint16_t)l;
    c->src = src;
    c->connected_ma = connected_ma;
    if (connected_ma == 0u || src == CLIMIT_SRC_NONE) {
        c->need = CLIMIT_NEED_UNKNOWN;
    } else {
        c->need = connected_ma > l ? CLIMIT_NEED_YES : CLIMIT_NEED_NO;
    }
}
