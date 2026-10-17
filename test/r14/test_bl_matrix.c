/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* All three release images against each hardware identity; interrupted transfer and retry.
 * Uses the existing bootloader NOR/controller fixture, but its actual bl_core is compiled
 * separately for BL_HW_ID 1, 2 and 3. The app fixture only triggers handover: app filtering
 * is checked separately in test_boundaries.c and test_ud_main.py. */
#define BL_TEST_NO_MAIN
#include "../../bootloader/test/test_bl.c"

static int load_image(const char *path, image_t *im)
{
    FILE *f = fopen(path, "rb");
    size_t n;
    if (!f) return 0;
    if (fread(im->desc, 1, UPD_DESC_BYTES, f) != UPD_DESC_BYTES) { fclose(f); return 0; }
    n = fread(im->body, 1, sizeof im->body, f);
    fclose(f);
    im->len = (uint32_t)n;
    return n == upd_get32(im->desc + UPD_D_BODY_LEN);
}

int main(int argc, char **argv)
{
    int i;
    sess_opts_t interrupted = SESS_DEFAULT;
    if (argc != 4) return 2;
    interrupted.passes = 0;
    interrupted.end_ms = 0;  /* announce, then only the first data page below */
    for (i = 1; i <= 3; i++) {
        uint32_t image_hw;
        board_reset_model();
        make_image(&im_old, 4096, V(0, 0, 1, 0xFF), BL_HW_ID, 1u << BL_HW_REV, BL_VERSION, 1414);
        if (!load_image(argv[i], &im_new)) return 2;
        image_hw = im_new.desc[UPD_D_HW_ID];
        CHECK((bl_desc_check(im_new.desc) == 0) == (image_hw == BL_HW_ID), "descriptor hardware match");
        factory(&im_old);
        board_power_on();
        host_session(&im_new, 14, &SESS_DEFAULT);
        host_after();
        printf("bootloader hw=%u release hw=%u: running old=%d new=%d flash violations=%d\n",
               BL_HW_ID, image_hw, running(&im_old), running(&im_new), fl_violations);
        CHECK(running(image_hw == BL_HW_ID ? &im_new : &im_old), "only matching image installs");
        CHECK(keep_areas_intact() && !fl_violations, "unrelated flash intact");
        if (image_hw == BL_HW_ID) {
            unsigned chunk;
            board_reset_model();
            factory(&im_old);
            board_power_on();
            host_session(&im_new, 15, &interrupted);
            for (chunk = 0; chunk < 8; chunk++)
                host_upd(UPD_DATA, 15, chunk, 3, 0, im_new.body + chunk * 32, chunk == 7 ? 2700 : 0);
            host_idle(4000000u);
            CHECK(!B.in_app && BS.mode == BL_ACT_RECOVERY, "incomplete body: recovery, no partial image runs");
            host_session(&im_new, 16, &SESS_DEFAULT);
            host_after();
            CHECK(running(&im_new) && keep_areas_intact() && !fl_violations, "fresh session recovers the release image");
            printf("bootloader hw=%u: interrupted after first page -> recovery; retry -> verified release running\n", BL_HW_ID);
        }
    }
    printf("R14 bootloader hardware %u: %d checks, %d failures\n", BL_HW_ID, checks, fails);
    return fails ? 1 : 0;
}
