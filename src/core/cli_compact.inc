/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include <stddef.h>
#include "cli.h"
#include "console_record.h"
#include "link_stats.h"
#include "../target.h"
#include "console_fields.inc"
#define REC(id, base, index) console_record(c->write, index, base, cf_##id)
void cli_say(cli_t *c, int z) { uint32_t v=(uint32_t)z; REC(X,&v,0); }
void cli_banner(cli_t *c) { REC(B,c->st,0); REC(i,c->st->uid,0); }
void cli_watch(cli_t *c) { REC(R,c->st,0); }
void evlog_boot(evlog_t *l, uint32_t keep, uint32_t prev_up_s)
{
    uint32_t i;
    if (!keep || l->magic != EVL_MAGIC) {
        uint8_t *p=(uint8_t *)l;
        for(i=0;i<sizeof(*l);i++) p[i]=0;
        l->magic=EVL_MAGIC;
    } else for(i=0;i<EVLOG_N;i++) l->e[i].t_s-=prev_up_s+1u;
}
uint32_t cli_poll(cli_t *c)
{
    uint32_t p=c->st_part;
    if(!p) return 0;
    c->st_part++;
    switch(p) {
    case 1: REC(S,c->set,0); break;
    case 2: REC(T,c->eff,0); break;
    case 3: REC(I,c->st,0); break;
    case 4: REC(C,c->st,0); break;
    case 5: REC(M,c->st,0); break;
    case 6: REC(A,&c->st->cal,0); break;
    case 7: case 8: case 9: case 10: case 11: REC(a,c->st->cal.ma[p-7],p-7); break;
    case 12: REC(H,c->st->hl,0); break;
    case 13: REC(L,&c->st->en->lt,0); break;
    case 14: REC(E,c->st->en,0); break;
    case 15: REC(P,c->st->pw,0); break;
    case 16: REC(V,c->st->dip,0); break;
    case 17: REC(O,c->scene,0); break;
    case 18: REC(F,&c->st->bc,0); break;
    case 19: REC(Q,(const void *)c->st->ovr,0); break;
    case 20: REC(D,(const void *)c->st->hwd,0); break;
    default:
        if(p<29) { REC(N,c->st->ev?&c->st->ev->e[p-21]:0,p-21); }
        else if(p<33) { REC(v,c->st->dip?&c->st->dip->log[p-29]:0,p-29); }
        else if(p==33) { REC(v,c->st->dip?&c->st->dip->deep:0,4); }
        else if(p==34) { REC(U,c->ul?c->ul->cfg:0,0); }
        else if(p==35) { REC(W,c->ul,0); }
        else if(p==36) { REC(s,c->ext,0); }
        else if(p<52) {
            uint32_t k=(p-37)/3;
            const ultra_light_t *l=c->ul?&c->ul->light[k]:0;
            switch((p-37)%3) {
            case 0: REC(G,l,k); break; case 1: REC(g,l,k); break;
            default: REC(J,l,k); break;
            }
        }
        else if(p==52) { REC(K,&line_stats,0); }
        else if(p==53) { REC(Y,c->st,0); }
#if MON_EXTRA
        else if(p==54) { REC(e,c->st->en,0); }
        else if(p<55+CALHIST_N) { REC(h,c->st->ch?&c->st->ch->e[p-55]:0,p-55); }
#endif
        else c->st_part=0;
        break;
    }
    return c->st_part!=0;
}
/* The production jig report is the one textual exception. Preserve its rounding,
 * clamping, separators and CRLF exactly; jig input pad processing is unchanged. */
#if BOARD_TESTJIG
#include "jig_line.inc"
void cli_test_report(cli_t *c,const int32_t ma[5],uint32_t fail,uint32_t cycle,uint32_t supply_mv) {
    jig_line(c->write,ma,fail,cycle,supply_mv,c->st->uid,c->st->version);
}
#endif
#undef REC
