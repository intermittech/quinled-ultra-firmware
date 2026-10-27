/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Compare this program's bytes against b7a03fc on each board. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
#include "cli.h"
static void output(const char *s) { fputs(s,stdout); }
static uint32_t next(uint32_t *s) { *s=*s*1664525u+1013904223u; return *s; }
int main(void) {
    cli_t c;cli_status_t st;uint32_t uid[3],state=1,k,j;int32_t ma[5];
#ifdef _WIN32
    _setmode(_fileno(stdout),_O_BINARY);
#endif
    memset(&c,0,sizeof(c));memset(&st,0,sizeof(st));c.write=output;c.st=&st;st.uid=uid;st.version="0.6.12-beta.63";
    for(k=0;k<5000;k++) {
        for(j=0;j<3;j++)uid[j]=next(&state);
        for(j=0;j<5;j++)ma[j]=(int32_t)next(&state);
        cli_test_report(&c,ma,k%32,next(&state),next(&state));
    }
    return 0;
}
