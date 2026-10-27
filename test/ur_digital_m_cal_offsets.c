/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "../src/app/main_ur_digital_m.c"
#include <stddef.h>
#define O(t,f) (unsigned)offsetof(t,f)
__attribute__((used,section(".rodata.uc_offs"))) const unsigned uc_offs[]={
    sizeof(udcal_t),sizeof(udcal_model_t),
    O(udcal_t,candidate),O(udcal_t,phase),O(udcal_t,error),O(udcal_t,pending),O(udcal_t,finish),
    O(udcal_t,duty),O(udcal_t,mask),O(udcal_t,k),O(udcal_t,pos),O(udcal_t,cap),O(udcal_t,stage),O(udcal_t,sampling),
    O(udcal_t,limit),O(udcal_t,now_ms),O(udcal_t,block_ms),O(udcal_t,start_ms),
    O(udcal_model_t,flags),O(udcal_model_t,count),O(udcal_model_t,epoch),
    O(udfx_t,e),O(udfx_t,pack),O(fx_pack_cfg_t,ord),O(udfx_t,calibration),O(udfx_t,begun),
    O(ws_enc_t,state),O(ws_enc_t,low_w),O(ws_enc_t,reset_w),O(ws_enc_t,ready),O(ws_enc_t,frames),
    O(ws_enc_t,lost),O(ws_enc_t,nbytes),O(ws_enc_t,i),O(ws_enc_t,w),
    O(dline_t,state),O(dline_t,wpos),O(dline_t,ch_on),O(dline_t,ch_win),O(dline_t,local),
    O(vsense_t,valid),O(vsense_t,mv),O(udc_t,mode),O(udfx_t,local),O(udfx_t,dark),
    O(udfx_t,want_stream),O(udfx_t,want_local),O(udfx_t,npx),O(udfx_t,chip),
    O(udfx_t,seg),sizeof(ux_seg_t),O(ux_seg_t,anchor),O(ux_seg_t,restarts),O(ws_chip_t,bpp),
    O(upd_app_t,run),O(upd_app_t,transparent),O(udfx_t,reconf)
};
