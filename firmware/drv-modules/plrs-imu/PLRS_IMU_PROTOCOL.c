#include "PLRS_IMU_PROTOCOL.h"

/*
 * Little-endian reads. The frame sits at unaligned offsets, so a value is
 * gathered byte by byte rather than read through a reinterpreted pointer.
 */

static uint16_t plrs_imu_read_u16_le(const uint8_t *b) {
    return (uint16_t)((uint16_t)b[0] | (uint16_t)b[1] << 8);
}

static uint32_t plrs_imu_read_u32_le(const uint8_t *b) {
    return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 |
           (uint32_t)b[3] << 24;
}

static float plrs_imu_read_f32_le(const uint8_t *b) {
    union {
        uint32_t u;
        float f;
    } pun = {.u = plrs_imu_read_u32_le(b)};
    return pun.f;
}

/*
 * Integrity.
 */

uint16_t plrs_imu_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = PLRS_IMU_CRC16_INIT;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ PLRS_IMU_CRC16_POLY)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/*
 * Framing.
 */

bool plrs_imu_cobs_decode(const uint8_t *block, size_t len, uint8_t *out,
                          size_t out_cap, size_t *out_len) {
    size_t i = 0;
    size_t n = 0;
    while (i < len) {
        uint8_t code = block[i++];
        if (code == PLRS_IMU_DELIMITER) {
            return false; // a zero cannot occur inside a COBS block
        }
        for (uint8_t j = 1; j < code; j++) {
            if (i >= len || n >= out_cap) {
                return false;
            }
            out[n++] = block[i++];
        }
        // a block shorter than 0xFF stands in for a zero, except the final one
        if (code != 0xFF && i < len) {
            if (n >= out_cap) {
                return false;
            }
            out[n++] = PLRS_IMU_DELIMITER;
        }
    }
    *out_len = n;
    return true;
}

/*
 * Messages.
 */

bool plrs_imu_heading_from_payload(const uint8_t *payload, size_t len,
                                   float *deg_out) {
    if (len != sizeof(float)) {
        return false;
    }
    *deg_out = plrs_imu_read_f32_le(payload);
    return true;
}

bool plrs_imu_attitude_from_payload(const uint8_t *payload, size_t len,
                                    PlrsImuAttitude *out) {
    if (len != 4 * sizeof(float) + 1) {
        return false;
    }
    out->heading_deg = plrs_imu_read_f32_le(&payload[0]);
    out->roll_deg = plrs_imu_read_f32_le(&payload[4]);
    out->pitch_deg = plrs_imu_read_f32_le(&payload[8]);
    out->yaw_rate_dps = plrs_imu_read_f32_le(&payload[12]);
    out->heading_valid = payload[16] != 0;
    return true;
}

bool plrs_imu_raw_attitude_from_payload(const uint8_t *payload, size_t len,
                                        PlrsImuRawAttitude *out) {
    if (len != 2 * sizeof(float)) {
        return false;
    }
    out->heel_deg = plrs_imu_read_f32_le(&payload[0]);
    out->yaw_rate_dps = plrs_imu_read_f32_le(&payload[4]);
    return true;
}

/*
 * Receiving.
 */

void plrs_imu_parser_reset(PlrsImuParser *self) {
    self->len = 0;
    self->overflow = false;
}

bool plrs_imu_parser_mid_frame(const PlrsImuParser *self) {
    return self->len > 0;
}

static PlrsImuStatus plrs_imu_parser_parse(PlrsImuParser *self, size_t block_len,
                                           PlrsImuFrame *out) {
    size_t body_len;
    if (!plrs_imu_cobs_decode(self->block, block_len, self->decoded,
                              sizeof(self->decoded), &body_len)) {
        return PLRS_IMU_ERR_MALFORMED;
    }
    if (body_len < PLRS_IMU_HEADER_BYTES + PLRS_IMU_CRC_BYTES) {
        return PLRS_IMU_ERR_MALFORMED;
    }
    if (self->decoded[0] != PLRS_IMU_PROTOCOL_VERSION) {
        return PLRS_IMU_ERR_WRONG_VERSION;
    }

    const size_t crc_at = body_len - PLRS_IMU_CRC_BYTES;
    const uint16_t want = plrs_imu_crc16(self->decoded, crc_at);
    const uint16_t got = plrs_imu_read_u16_le(&self->decoded[crc_at]);
    if (want != got) {
        return PLRS_IMU_ERR_BAD_CRC;
    }

    out->msg_id = self->decoded[1];
    out->seq = self->decoded[2];
    out->payload = &self->decoded[PLRS_IMU_HEADER_BYTES];
    out->payload_len = crc_at - PLRS_IMU_HEADER_BYTES;
    return PLRS_IMU_FRAME_OK;
}

PlrsImuStatus plrs_imu_parser_feed(PlrsImuParser *self, uint8_t byte,
                                   uint32_t now_ms, PlrsImuFrame *out) {
    if (self->len > 0 &&
        now_ms - self->last_advance_ms > PLRS_IMU_FRAME_TIMEOUT_MS) {
        plrs_imu_parser_reset(self);
    }
    self->last_advance_ms = now_ms;

    if (byte != PLRS_IMU_DELIMITER) {
        if (self->len < sizeof(self->block)) {
            self->block[self->len++] = byte;
        } else {
            self->overflow = true;
        }
        return PLRS_IMU_FRAME_NONE;
    }

    const bool overflowed = self->overflow;
    const size_t block_len = self->len;
    plrs_imu_parser_reset(self);
    if (block_len == 0) {
        return PLRS_IMU_FRAME_NONE;
    }
    if (overflowed) {
        return PLRS_IMU_ERR_MALFORMED;
    }
    return plrs_imu_parser_parse(self, block_len, out);
}
