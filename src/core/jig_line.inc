/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The production jig's `jig` line (core/testjig.h), the console's one textual exception on every board with the test jig:
 * cli_compact.inc's cli_test_report (boards 1 / 2) and main_ur_digital_m.c. Its rounding, clamping, separators and CRLF
 * are exact (5000 reports per analog board byte-equal to b7a03fc: test/console_jig_bytes.c). Included once per image. */
static char *dec(char *p,uint32_t v,uint32_t width) {
    char t[10]; uint32_t n=0;
    do { t[n++]=(char)('0'+v%10); v/=10; } while(v||n<width);
    while(n) *p++=t[--n];
    return p;
}
/* the line from a template: 1 the next value in decimal, 2 x.yy (the next value in thousandths, rounded half up), 3 PASS /
 * FAIL, 4 the version */
static void jig_line(void (*write)(const char *),const int32_t ma[5],uint32_t fail,uint32_t cycle,uint32_t supply_mv,
                     const uint32_t *uid,const char *version) {
    static const char f[]="jig cycle=\1 \3 fail=\1 a=\2/\2/\2/\2/\2 supply=\2 uid=\1,\1,\1 fw=\4\r\n";
    char b[240],*p=b; const char *t,*s; uint32_t v[11],*a=v,i,q;
    v[0]=cycle; v[1]=fail;
    for(i=0;i<5;i++) v[2+i]=ma[i]>0?(uint32_t)ma[i]:0;
    v[7]=supply_mv; v[8]=uid[0]; v[9]=uid[1]; v[10]=uid[2];
    for(t=f;*t;t++) {
        if(*t==1) p=dec(p,*a++,1);
        else if(*t==2) { q=*a++; q=q/10+(q%10>=5); p=dec(p,q/100,1); *p++='.'; p=dec(p,q%100,2); }
        else if(*t>4) *p++=*t;
        else { for(s=*t==3?(fail?"FAIL":"PASS"):version;*s;) *p++=*s++; }
    }
    *p=0; write(b);
}
