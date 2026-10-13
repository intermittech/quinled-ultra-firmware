/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
// The ESPHome component's update sender (quinled-ultra-esphome/components/dig2analog_ultra/update_session.h) against
// the real bootloader logic: its exact frames and pauses go through the flash / board model of test_bl.c.
// Built by build_bl_test.cmd when it finds quinled-ultra-esphome (ULTRA_ESPHOME, or next to the firmware).
#define BL_TEST_NO_MAIN
#include "test_bl.c"

#include "fw_image_cache.h"
#include "fw_manifest.h"
#include "fw_version.h"
#include "update_session.h"

#include <string>

using namespace esphome::dig2analog_ultra;

static const uint32_t FRAME_US = 384u * 1350u / 1000u + 300u;   // the hub's worst bit (700 + 650 ns) + reset

// sends a whole session from the sender; returns its duration on the wire (us)
static uint64_t send_session(const image_t *im, uint8_t session, const UpdateSession::Options &o)
{
    static uint8_t img[UPD_DESC_BYTES + UPD_APP_BODY_MAX];
    UpdateSession us;
    uint8_t f[UPD_FRAME_BYTES];
    uint32_t pause = 0, frames = 0;
    uint64_t t0 = B.now_us;
    memcpy(img, im->desc, UPD_DESC_BYTES);
    memcpy(img + UPD_DESC_BYTES, im->body, im->len);
    CHECK(us.begin(img, UPD_DESC_BYTES + im->len, session, o, FRAME_US), "begin");
    while (us.next(f, (uint32_t) B.now_us, &pause)) {
        host_frame_bits(f, UPD_FRAME_BYTES, pause + 50u);    // + the extra 50 us of the worst bit time
        frames++;
    }
    CHECK(us.done() && frames == us.total_frames() && us.progress() == 1.0f, "sender: %u frames, total %u",
          (unsigned) frames, (unsigned) us.total_frames());
    return B.now_us - t0;
}

static void test_sender_sessions(void)
{
    UpdateSession::Options o;
    printf("ESPHome sender -> bootloader (30.5 KB and a full slot, 3 passes, the hub's default timing)\n");
    board_reset_model();
    make_image(&im_old, 30548, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 201);
    make_image(&im_new, 30548, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 202);
    factory(&im_old);
    board_power_on();
    {
        uint64_t us = send_session(&im_new, 0x31, o);
        printf("  30.5 KB: %.2f s on the wire\n", us / 1e6);
        CHECK(us < 4500000u, "session %.2f s (report: 4.1 s)", us / 1e6);
    }
    host_after();
    CHECK(running(&im_new), "installed from the sender's frames");

    board_reset_model();
    make_image(&im_old, 30548, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 203);
    make_image(&im_new, UPD_APP_BODY_MAX, V(0, 6, 0, 0xFF), 1, REV_OK, 1, 204);
    factory(&im_old);
    board_power_on();
    {
        uint64_t us = send_session(&im_new, 0x32, o);
        printf("  full slot (%u B): %.2f s on the wire\n", (unsigned) UPD_APP_BODY_MAX, us / 1e6);
    }
    host_after();
    CHECK(running(&im_new), "full-slot image installed");

    printf("  with 1 %% frame loss, 50 boards' lead-in, force a downgrade\n");
    board_reset_model();
    make_image(&im_old, 30548, V(0, 6, 0, 0xFF), 1, REV_OK, 1, 205);
    make_image(&im_new, 30548, V(0, 5, 9, 0xFF), 1, REV_OK, 1, 206);
    factory(&im_old);
    board_power_on();
    loss_ppm = 10000;
    o.force = true;
    o.boards = 50;
    send_session(&im_new, 0x33, o);
    loss_ppm = 0;
    host_after();
    CHECK(running(&im_new), "forced downgrade installed with loss");

    printf("  ABORT and PING frames from the sender\n");
    board_reset_model();
    factory(&im_old);
    board_power_on();
    {
        UpdateSession s;
        uint8_t f[UPD_FRAME_BYTES];
        uint32_t pause, i;
        static uint8_t img[UPD_DESC_BYTES + 30548];
        UpdateSession::Options q;
        memcpy(img, im_new.desc, UPD_DESC_BYTES);
        memcpy(img + UPD_DESC_BYTES, im_new.body, 30548);
        q.force = true;
        s.begin(img, sizeof(img), 0x34, q, FRAME_US);
        for (i = 0; i < 1500u && s.next(f, (uint32_t) B.now_us, &pause); i++) {
            host_frame_bits(f, UPD_FRAME_BYTES, pause);
        }
        s.abort_frame(f, (uint32_t) B.now_us);
        host_frame_bits(f, UPD_FRAME_BYTES, 0);
        host_idle(10000u);
        CHECK(!B.in_app && BS.mode == BL_ACT_RECOVERY, "ABORT mid-session: recovery");
        UpdateSession::ping_frame(f, 0);
        CHECK(upd_check_frame(f) == 0 && f[UPD_B_TYPE] == UPD_PING, "PING frame valid");
    }
    CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules");
}

static void test_versions(void)
{
    static const char *const order[] = { "0.5.0", "0.5.1-alpha.0", "0.5.1-alpha.12", "0.5.1-beta.1", "0.5.1-rc.2",
                                         "0.5.1", "0.6.0-alpha.1", "1.0.0", "255.255.255" };
    static const char *const bad[] = { "", "1.2", "1.2.3.4", "1.2.256", "1.2.3-gamma.1", "1.2.3-beta", "1.2.3-beta.64",
                                       "1.2.3 ", "v1.2.x" };
    uint32_t prev = 0, i, w;
    printf("firmware versions (semver <-> descriptor word)\n");
    for (i = 0; i < sizeof(order) / sizeof(order[0]); i++) {
        CHECK(parse_fw_version(order[i], w), "parses %s", order[i]);
        CHECK(format_fw_version(w) == order[i], "round trip %s -> %s", order[i], format_fw_version(w).c_str());
        CHECK(i == 0 || w > prev, "%s sorts after %s", order[i], order[i - 1]);
        prev = w;
    }
    CHECK(parse_fw_version("v0.5.0", w) && w == V(0, 5, 0, 0xFF), "v prefix");
    for (i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        CHECK(!parse_fw_version(bad[i], w), "rejects '%s'", bad[i]);
    }
}

// the update entity's logic without ESPHome (fw_manifest.h): channels, URLs, the image choice, the image checks
static std::string hex_of(const uint8_t *b, size_t n)
{
    static const char *const H = "0123456789abcdef";
    std::string s;
    for (size_t i = 0; i < n; i++) {
        s += H[b[i] >> 4];
        s += H[b[i] & 15];
    }
    return s;
}

static void test_manifest(void)
{
    static uint8_t img[UPD_DESC_BYTES + 8192];
    uint8_t sha[32], bad_sha[32];
    FwManifest m;
    FwImageInfo a, b;
    uint32_t i, size;
    printf("ESPHome update entity: manifest, image checks, offer rule (fw_manifest.h)\n");
    for (i = 0; i < 3; i++) {
        const uint8_t ch = (uint8_t) (1u << i);
        CHECK(fw_channel_from_name(fw_channel_name(ch)) == ch, "channel %u round trip", (unsigned) ch);
    }
    CHECK(fw_channel_name(8) == nullptr && fw_channel_from_name("nightly") == 0, "unknown channels");
    CHECK(fw_manifest_url("OWNER/REPO", FW_CH_BETA) == "https://raw.githubusercontent.com/OWNER/REPO/channels/beta.json",
          "GitHub manifest URL");
    CHECK(fw_manifest_url("http://nas.local/fw/{channel}.json", FW_CH_ALPHA) == "http://nas.local/fw/alpha.json",
          "own server, {channel}");
    CHECK(fw_manifest_url("https://x.org/m.json", FW_CH_STABLE) == "https://x.org/m.json", "own fixed manifest");
    CHECK(fw_resolve_url("https://x.org/a/b.json", "img.bin") == "https://x.org/a/img.bin" &&
              fw_resolve_url("https://x.org/a/b.json", "/c/img.bin") == "https://x.org/c/img.bin" &&
              fw_resolve_url("https://x.org/a/b.json", "https://y.org/i.bin") == "https://y.org/i.bin",
          "image URLs: relative, root-relative, absolute");

    a.hw_id = 1;
    a.hw_rev_mask = 0x8;   // revision 3 (rev 0.4)
    b.hw_id = 1;
    b.hw_rev_mask = 0x1;   // another revision (never built)
    m.images.push_back(b);
    m.images.push_back(a);
    CHECK(fw_pick_image(m, 1, UPD_REV_D2A_R04) == 1 && fw_pick_image(m, 1, 0) == 0, "image per board revision");
    CHECK(fw_pick_image(m, UPD_HW_ID_DP_ULTRA_RX, UPD_REV_D2A_R04) == -1 && fw_pick_image(m, 1, 1) == -1 &&
              fw_pick_image(m, 1, 2) == -1,
          "no image for other hardware / revision");
    m.channel = "beta";
    m.version = "0.6.0-beta.1";
    CHECK(fw_check_manifest(m, FW_CH_BETA) == nullptr && m.version_word == V(0, 6, 0, 0x41), "manifest ok");
    CHECK(fw_check_manifest(m, FW_CH_STABLE) != nullptr, "manifest of another channel");
    m.version = "0.6";
    CHECK(fw_check_manifest(m, FW_CH_BETA) != nullptr, "manifest with a bad version");

    // the image checks (SHA-256 itself is mbedTLS on the ESP32: here a stand-in hash)
    make_image(&im_new, 8192, V(0, 6, 0, 0x41), 1, REV_OK, 1, 99);
    memcpy(img, im_new.desc, UPD_DESC_BYTES);
    memcpy(img + UPD_DESC_BYTES, im_new.body, im_new.len);
    size = UPD_DESC_BYTES + im_new.len;
    for (i = 0; i < 32; i++) {
        sha[i] = (uint8_t) (i * 7 + 1);
        bad_sha[i] = sha[i];
    }
    bad_sha[31] ^= 1;
    a.size = size;
    a.sha256 = hex_of(sha, 32);
    CHECK(fw_check_image(img, size, sha, a, V(0, 6, 0, 0x41), UPD_REV_D2A_R04) == nullptr, "image ok");
    CHECK(fw_check_image(img, size, bad_sha, a, V(0, 6, 0, 0x41), UPD_REV_D2A_R04) != nullptr, "SHA-256 differs");
    CHECK(fw_check_image(img, size - 4, sha, a, V(0, 6, 0, 0x41), UPD_REV_D2A_R04) != nullptr, "size differs");
    CHECK(fw_check_image(img, size, sha, a, V(0, 6, 0, 0x42), UPD_REV_D2A_R04) != nullptr, "version differs from the manifest");
    CHECK(fw_check_image(img, size, sha, a, V(0, 6, 0, 0x41), 0) != nullptr, "revision not in the descriptor");
    img[UPD_D_HW_ID] ^= 3;
    CHECK(fw_check_image(img, size, sha, a, V(0, 6, 0, 0x41), UPD_REV_D2A_R04) != nullptr, "descriptor CRC broken");
    img[UPD_D_HW_ID] ^= 3;
    a.sha256[5] = 'x';
    CHECK(fw_check_image(img, size, sha, a, V(0, 6, 0, 0x41), UPD_REV_D2A_R04) != nullptr, "manifest SHA-256 not hex");

    CHECK(fw_update_available(0, V(0, 5, 0, 0xFF)), "installed unknown: offered");
    CHECK(fw_update_available(V(0, 5, 0, 0x41), V(0, 5, 0, 0xFF)), "beta installed, stable out: offered");
    CHECK(!fw_update_available(V(0, 5, 0, 0xFF), V(0, 5, 0, 0xFF)), "same: not offered");
    CHECK(!fw_update_available(V(0, 6, 0, 0x03), V(0, 5, 0, 0xFF)), "alpha of a newer version, stable older: not offered");
    CHECK(!fw_update_available(V(0, 5, 0, 0xFF), 0), "no release: not offered");
    CHECK(fw_ping_burst_frames(10) >= 50 && FW_PING_DARK_TAIL >= 2,
          "PING burst: > 4.7 frames per board (firmware wire test), >= 2 DARK frames after it");
}

// the image shared by several lines (fw_image_cache.h), as fw_update.cpp uses it
static int cache_frees;
static void cache_free(uint8_t *img, size_t size)
{
    (void) size;
    cache_frees++;
    delete[] img;
}

static void test_image_cache(void)
{
    const std::string sha_a(64, 'a'), sha_b(64, 'b');
    FwImageCache c(cache_free);
    uint8_t *a = new uint8_t[1000], *b = new uint8_t[1000];
    printf("ESPHome update entity: one download shared by several lines (fw_image_cache.h)\n");
    cache_frees = 0;
    CHECK(c.acquire(sha_a, 1000) == nullptr, "empty cache: a download");
    CHECK(c.put(sha_a, a, 1000) && c.users() == 1, "downloaded image cached, one user");
    CHECK(c.acquire(std::string(64, 'A'), 1000) == a && c.users() == 2, "same SHA-256 (any case): shared");
    CHECK(c.acquire(sha_b, 1000) == nullptr && c.acquire(sha_a, 999) == nullptr, "other SHA-256 / size: not shared");
    CHECK(!c.put(sha_b, b, 1000) && cache_frees == 0, "another image while this one is in use: not cached");
    delete[] b;
    c.release(1000);
    c.release(1000);
    c.trim();
    CHECK(c.users() == 0 && cache_frees == 1 && c.empty(), "trim before another download frees an unused image");

    a = new uint8_t[1000];
    c.put(sha_a, a, 1000);
    c.release(5000);
    c.service(64999, 60000);
    CHECK(!c.empty(), "kept during the hold time (several lines)");
    CHECK(c.acquire(sha_a, 1000) == a, "a line queued behind takes it within the hold time");
    c.service(200000, 60000);
    CHECK(!c.empty(), "never freed while in use");
    c.release(200000);
    c.service(260000, 60000);
    CHECK(c.empty() && cache_frees == 2, "freed 60 s after its last user");
    a = new uint8_t[1000];
    c.put(sha_a, a, 1000);
    c.release(300000);
    c.service(300000, 0);
    CHECK(c.empty() && cache_frees == 3, "one line (hold 0): freed at once");

    // "update all" on 3 lines, installs queued one after the other: one download
    int downloads = 0;
    for (int line = 0; line < 3; line++) {
        uint8_t *img = c.acquire(sha_b, 2000);
        if (img == nullptr) {
            c.trim();
            img = new uint8_t[2000];
            downloads++;
            CHECK(c.put(sha_b, img, 2000), "line %d: cached", line);
        }
        c.release(400000u + 7000u * (uint32_t) line);    // its session ends 7 s later
        c.service(400000u + 7000u * (uint32_t) line + 100u, 60000);
    }
    CHECK(downloads == 1, "3 lines, %d download(s)", downloads);
    c.service(1000000, 60000);
    CHECK(c.empty() && cache_frees == 4, "freed after the last line");
}

// a release image against its mkimage.py .json (the real descriptor through fw_check_image)
static void test_release_image(const char *path, const uint8_t *buf, size_t n)
{
    std::string json_path = path, json;
    FwImageInfo info;
    uint8_t sha[32];
    uint32_t version = 0;
    json_path = json_path.substr(0, json_path.rfind('.')) + ".json";
    FILE *f = fopen(json_path.c_str(), "rb");
    if (f == nullptr) {
        printf("  (no %s: image check skipped)\n", json_path.c_str());
        return;
    }
    char tmp[4096];
    size_t got = fread(tmp, 1, sizeof(tmp) - 1, f);
    fclose(f);
    tmp[got] = 0;
    json = tmp;
    auto field = [&json](const char *key) {
        size_t at = json.find(std::string("\"") + key + "\"");
        if (at == std::string::npos)
            return std::string();
        at = json.find(':', at) + 1;
        while (json[at] == ' ' || json[at] == '"')
            at++;
        size_t end = at;
        while (end < json.size() && json[end] != '"' && json[end] != ',' && json[end] != '\n' && json[end] != '}')
            end++;
        return json.substr(at, end - at);
    };
    info.hw_id = (uint8_t) atoi(field("hw_id").c_str());
    info.hw_rev_mask = (uint16_t) atoi(field("hw_rev_mask").c_str());
    info.size = (uint32_t) atoi(field("size").c_str());
    info.sha256 = field("sha256");
    CHECK(parse_fw_version(field("version"), version), "release .json version %s", field("version").c_str());
    CHECK(fw_hex_to_bytes(info.sha256, sha, 32), "release .json sha256");   // computed by mkimage (hashlib)
    CHECK(fw_check_image(buf, n, sha, info, version, UPD_REV_D2A_R04) == nullptr, "release image passes the ESP32's checks: %s",
          fw_check_image(buf, n, sha, info, version, UPD_REV_D2A_R04) ? fw_check_image(buf, n, sha, info, version, UPD_REV_D2A_R04) : "ok");
}

int main(void)
{
    const char *img = getenv("BL_TEST_IMAGE");
    test_versions();
    test_manifest();
    test_image_cache();
    test_sender_sessions();
    if (img != 0 && *img != 0) {
        FILE *f = fopen(img, "rb");
        static uint8_t buf[UPD_DESC_BYTES + UPD_APP_BODY_MAX];
        size_t n = f ? fread(buf, 1, sizeof(buf), f) : 0;
        UpdateSession::Options o;
        printf("release image through the sender: %s\n", img);
        if (f) {
            fclose(f);
        }
        board_reset_model();
        make_image(&im_old, 4096, V(0, 0, 1, 0xFF), 1, REV_OK, 1, 207);
        memcpy(im_new.desc, buf, UPD_DESC_BYTES);
        im_new.len = (uint32_t) (n - UPD_DESC_BYTES);
        memcpy(im_new.body, buf + UPD_DESC_BYTES, im_new.len);
        factory(&im_old);
        board_power_on();
        printf("  %.2f s on the wire\n", send_session(&im_new, 0x35, o) / 1e6);
        host_after();
        CHECK(running(&im_new), "release image installed");
        test_release_image(img, buf, n);
    }
    printf("\n%d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
