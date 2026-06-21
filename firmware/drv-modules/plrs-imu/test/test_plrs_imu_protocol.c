/**
 * Host tests for the PLRS-IMU rudder link protocol core. No HAL, no framework:
 * build with the Makefile here and run the binary.
 */

#include "PLRS_IMU_PROTOCOL.h"

#include <stdio.h>
#include <string.h>

static int g_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);             \
            g_failures++;                                                      \
        }                                                                      \
    } while (0)

/*
 * Test-only helpers: an independent COBS encoder and frame builder, so the
 * decoder is checked against something it does not share code with.
 */

static size_t cobs_encode(const uint8_t *data, size_t len, uint8_t *out) {
    size_t out_i = 0;
    size_t code_i = out_i++;
    uint8_t code = 1;
    for (size_t i = 0; i < len; i++) {
        if (data[i] != PLRS_IMU_DELIMITER) {
            out[out_i++] = data[i];
            code++;
            if (code != 0xFF) {
                continue;
            }
        }
        out[code_i] = code;
        code_i = out_i++;
        code = 1;
    }
    out[code_i] = code;
    out[out_i++] = PLRS_IMU_DELIMITER;
    return out_i;
}

static void put_f32_le(float f, uint8_t out[4]) {
    union {
        float f;
        uint32_t u;
    } pun = {.f = f};
    out[0] = (uint8_t)(pun.u & 0xFF);
    out[1] = (uint8_t)((pun.u >> 8) & 0xFF);
    out[2] = (uint8_t)((pun.u >> 16) & 0xFF);
    out[3] = (uint8_t)((pun.u >> 24) & 0xFF);
}

// Build a frame, optionally corrupting the CRC, into out; returns its length.
static size_t build_frame(uint8_t ver, uint8_t msg_id, uint8_t seq,
                          const uint8_t *payload, size_t payload_len,
                          bool corrupt_crc, uint8_t *out) {
    uint8_t body[PLRS_IMU_MAX_DATA];
    size_t n = 0;
    body[n++] = ver;
    body[n++] = msg_id;
    body[n++] = seq;
    for (size_t i = 0; i < payload_len; i++) {
        body[n++] = payload[i];
    }
    uint16_t crc = plrs_imu_crc16(body, n);
    if (corrupt_crc) {
        crc ^= 0x0001u;
    }
    body[n++] = (uint8_t)(crc & 0xFF);
    body[n++] = (uint8_t)(crc >> 8);
    return cobs_encode(body, n, out);
}

// Feed every byte; return the last non-NONE status (the one at the delimiter).
static PlrsImuStatus feed_all(PlrsImuParser *p, const uint8_t *bytes,
                              size_t len, PlrsImuFrame *out) {
    PlrsImuStatus last = PLRS_IMU_FRAME_NONE;
    for (size_t i = 0; i < len; i++) {
        PlrsImuStatus s = plrs_imu_parser_feed(p, bytes[i], 0, out);
        if (s != PLRS_IMU_FRAME_NONE) {
            last = s;
        }
    }
    return last;
}

/*
 * Tests.
 */

static void test_crc16_known_answer(void) {
    const uint8_t check[] = "123456789";
    CHECK(plrs_imu_crc16(check, sizeof(check) - 1) == 0x29B1);
}

static void test_cobs_decode_doc_vector(void) {
    const uint8_t block[] = {0x03, 0x11, 0x22, 0x02, 0x33};
    const uint8_t want[] = {0x11, 0x22, 0x00, 0x33};
    uint8_t out[PLRS_IMU_MAX_DATA];
    size_t n = 0;
    CHECK(plrs_imu_cobs_decode(block, sizeof(block), out, sizeof(out), &n));
    CHECK(n == sizeof(want));
    CHECK(memcmp(out, want, n) == 0);
}

static void test_heading_roundtrip(void) {
    PlrsImuParser p = {0};
    uint8_t payload[4];
    const float deg = 123.5f;
    put_f32_le(deg, payload);

    uint8_t frame[PLRS_IMU_MAX_FRAME];
    size_t len = build_frame(PLRS_IMU_PROTOCOL_VERSION, PLRS_IMU_MSG_HEADING, 7,
                             payload, sizeof(payload), false, frame);

    PlrsImuFrame out;
    CHECK(feed_all(&p, frame, len, &out) == PLRS_IMU_FRAME_OK);
    CHECK(out.msg_id == PLRS_IMU_MSG_HEADING);
    CHECK(out.seq == 7);
    CHECK(out.payload_len == sizeof(payload));

    float got = 0.0f;
    CHECK(plrs_imu_heading_from_payload(out.payload, out.payload_len, &got));
    CHECK(got == deg);
}

static void test_wrong_version(void) {
    PlrsImuParser p = {0};
    uint8_t payload[4];
    put_f32_le(0.0f, payload);
    uint8_t frame[PLRS_IMU_MAX_FRAME];
    size_t len = build_frame(PLRS_IMU_PROTOCOL_VERSION + 1, PLRS_IMU_MSG_HEADING,
                             0, payload, sizeof(payload), false, frame);
    PlrsImuFrame out;
    CHECK(feed_all(&p, frame, len, &out) == PLRS_IMU_ERR_WRONG_VERSION);
}

static void test_bad_crc(void) {
    PlrsImuParser p = {0};
    uint8_t payload[4];
    put_f32_le(45.0f, payload);
    uint8_t frame[PLRS_IMU_MAX_FRAME];
    size_t len = build_frame(PLRS_IMU_PROTOCOL_VERSION, PLRS_IMU_MSG_HEADING, 0,
                             payload, sizeof(payload), true, frame);
    PlrsImuFrame out;
    CHECK(feed_all(&p, frame, len, &out) == PLRS_IMU_ERR_BAD_CRC);
}

static void test_overflow(void) {
    PlrsImuParser p = {0};
    PlrsImuFrame out;
    for (size_t i = 0; i < PLRS_IMU_MAX_FRAME + 10; i++) {
        plrs_imu_parser_feed(&p, 0x42, 0, &out);
    }
    CHECK(plrs_imu_parser_feed(&p, PLRS_IMU_DELIMITER, 0, &out) ==
          PLRS_IMU_ERR_MALFORMED);
}

static void test_resync_after_garbage(void) {
    PlrsImuParser p = {0};
    PlrsImuFrame out;

    const uint8_t garbage[] = {0x11, 0x22, 0x33};
    for (size_t i = 0; i < sizeof(garbage); i++) {
        plrs_imu_parser_feed(&p, garbage[i], 0, &out);
    }
    plrs_imu_parser_feed(&p, PLRS_IMU_DELIMITER, 0, &out); // ends the garbage

    uint8_t payload[4];
    put_f32_le(10.0f, payload);
    uint8_t frame[PLRS_IMU_MAX_FRAME];
    size_t len = build_frame(PLRS_IMU_PROTOCOL_VERSION, PLRS_IMU_MSG_HEADING, 1,
                             payload, sizeof(payload), false, frame);
    CHECK(feed_all(&p, frame, len, &out) == PLRS_IMU_FRAME_OK);
    CHECK(out.seq == 1);
}

static void test_two_frames_keep_seq(void) {
    PlrsImuParser p = {0};
    PlrsImuFrame out;
    uint8_t payload[4];
    put_f32_le(0.0f, payload);
    uint8_t frame[PLRS_IMU_MAX_FRAME];

    size_t len = build_frame(PLRS_IMU_PROTOCOL_VERSION, PLRS_IMU_MSG_HEADING, 5,
                             payload, sizeof(payload), false, frame);
    CHECK(feed_all(&p, frame, len, &out) == PLRS_IMU_FRAME_OK);
    CHECK(out.seq == 5);

    len = build_frame(PLRS_IMU_PROTOCOL_VERSION, PLRS_IMU_MSG_HEADING, 9,
                      payload, sizeof(payload), false, frame);
    CHECK(feed_all(&p, frame, len, &out) == PLRS_IMU_FRAME_OK);
    CHECK(out.seq == 9);
}

int main(void) {
    test_crc16_known_answer();
    test_cobs_decode_doc_vector();
    test_heading_roundtrip();
    test_wrong_version();
    test_bad_crc();
    test_overflow();
    test_resync_after_garbage();
    test_two_frames_keep_seq();

    if (g_failures == 0) {
        printf("all tests passed\n");
        return 0;
    }
    printf("%d failure(s)\n", g_failures);
    return 1;
}
