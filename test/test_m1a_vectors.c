/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Compile with the pinned protocol test-vectors directory on the include path.
 * Exercises firmware implementations; no reference decoder is substituted. */
#include <stdio.h>
#include <string.h>
#include "../src/core/ultra_protocol.h"
#include "../src/core/upd_app.h"
#include "../src/core/ws_decode.h"
#include "../src/core/rc.h"
#include "shared_vectors.h"
static unsigned checks, failures;
#define CHECK(x) do { ++checks; if(!(x)) { ++failures; printf("FAIL %s:%u %s\n",name,(unsigned)__LINE__,#x); } } while(0)
static uint16_t samples[2048];
static uint8_t decoded[512];
static const ws_timing_t timing=WS_TIMING_DEFAULT;
static void waveform(const uint8_t *b,unsigned n,const char *name)
{
    unsigned k,j,pos=32;
    memset(samples,0,sizeof samples);
    for(k=0;k<n*8u;++k) {
        unsigned high=(b[k/8]&(128u>>(k%8)))?28u:12u;
        for(j=0;j<high;++j) samples[(pos+j)/16]|=(uint16_t)(32768u>>((pos+j)%16));
        pos+=48;
    }
    CHECK(ws_decode_frame(samples,pos,n*8u,&timing,decoded)==(int)(n*8u));
    CHECK(!memcmp(b,decoded,n));
    CHECK(ws_rise_frame(samples,pos,n*8u,decoded)==(int)(n*8u));
    CHECK(!memcmp(b,decoded,n));
}
int main(void)
{
    unsigned k,i,wire=0;
    const char *name="layouts";
    CHECK(sizeof(upd_target_t)==16 && sizeof(upd_bl_info_t)==32 && sizeof(upd_descriptor_t)==64);
    for(k=0;k<ULTRA_SHARED_VECTOR_COUNT;++k) {
        const ultra_shared_vector_t *v=&ultra_shared_vectors[k];
        const uint8_t *b=v->bytes; uint8_t out[256];
        name=v->name;
        if(!strcmp(v->kind,"slot")) {
            CHECK(!ultra_slot_check(b)); memcpy(out,b,32);out[29]^=1;CHECK(ultra_slot_check(out));
            waveform(b,32,name);++wire;
        } else if(!strcmp(v->kind,"header")) {
            uint32_t t;CHECK(!ultra_parse_header(b,&t));
            ultra_build_header_flags(out,t,ultra_header_flags(b));CHECK(!memcmp(out,b,8));
            waveform(b,8,name);++wire;
        } else if(!strcmp(v->kind,"answer")) {
            uint8_t poll[32]={0};CHECK(!ultra_answer_check(b));
            poll[2]=b[3];poll[3]=b[2]>>4;
            memset(out,0,32);memcpy(out+6,b+6,24);
            rc_answer(out,poll,b[1],b[2]&15u,ultra_get16(b+4));CHECK(!memcmp(out,b,32));
        } else if(!strcmp(v->kind,"update")) {
            CHECK(!upd_check_frame(b));
            upd_build_frame(out,b[0],b[1],ultra_get16(b+2),b[4],b[5],b+6,ultra_get32(b+43));
            CHECK(!memcmp(out,b,48));out[42]^=1;CHECK(upd_check_frame(out));
            waveform(b,48,name);++wire;
        } else if(!strcmp(v->kind,"descriptor")) {
            unsigned at=b[1],n=0;CHECK(b[0]==1 && ultra_get16(b+2)==v->size);
            CHECK(ultra_crc16(b+6,v->size-6)==ultra_get16(b+4));
            while(at<v->size) {CHECK(at+2u<=v->size);at+=2u+b[at+1];++n;}
            CHECK(at==v->size && n==b[8]);
        } else if(!strcmp(v->kind,"target")) {
            upd_target_t t;memcpy(&t,b,16);CHECK(v->size==sizeof t && !memcmp(t.bytes,b,16));
        } else if(!strcmp(v->kind,"boot_info")) {
            upd_bl_info_t info;memcpy(&info,b,32);
            CHECK(!memcmp(info.magic,"QDB2",4) && !memcmp(info.target.bytes,b+5,16));
            CHECK(!memcmp(&info.geometry,b+21,3) && info.entry_10ms==b[24]);
        } else if(!strcmp(v->kind,"image")) {
            upd_target_t t={{0}};upd_bl_info_t bl={{'Q','D','B','2'},0};
            upd_app_info_t info;upd_app_t app;uint8_t ping[48],page[24]={0};
            memcpy(t.bytes,b+8,7);t.bytes[7]=b[6];
            memcpy(t.bytes+8,b+28,4);memcpy(t.bytes+12,b+20,4);
            CHECK(!upd_desc_check(b,&t,b[15]));
            for(i=0;i<64;++i) {memcpy(out,b,64);out[i]^=1;CHECK(upd_desc_check(out,&t,b[15]));}
            bl.bl_version=b[15];bl.target=t;
            memset(out,255,sizeof out);memcpy(out,b,64);
            out[64]=(uint8_t)UPD_ST_VERIFIED;out[65]=(uint8_t)(UPD_ST_VERIFIED>>8);
            upd_app_read_info(&info,out,(const uint8_t *)&bl);CHECK(info.desc_ok && !info.confirmed);
            upd_app_page(&info,out,page);CHECK(page[0]==1 && page[1]==ULTRA_FU_F_VERIFIED);
            CHECK(!memcmp(page+4,b+16,12) && !memcmp(page+16,b+60,4));
            upd_app_init(&app,&info);upd_app_frame_end(&app,1,0,0);
            upd_build_frame(ping,UPD_PING,7,0,0,0,0,0);
            CHECK(upd_app_frame(&app,ping,1)==UPD_APP_NONE && !app.ping_seen);
            upd_build_frame(ping,UPD_PING,0,0,0,0,0,0);
            CHECK(upd_app_frame(&app,ping,2)==UPD_APP_NONE);   /* startup checks not done yet: no confirmation */
            app.startup_ok=1;                                   /* as main.c after its board identity check */
            CHECK(upd_app_frame(&app,ping,3)==UPD_APP_CONFIRM);
        } else CHECK(0);
    }
    printf("M1a firmware: %u vectors; %u sampled frames x 2 decoders; %u checks, %u failures\n",
           (unsigned)ULTRA_SHARED_VECTOR_COUNT,wire,checks,failures);
    return failures!=0;
}
