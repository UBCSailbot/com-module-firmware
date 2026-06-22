/**
 * PLRS-IMU rudder link protocol. COBS-framed, little-endian.
 *
 * See the PLRS-IMU repository's docs/rudder_link.md for the wire format.
 */

#ifndef PLRS_IMU_PROTOCOL_H_
#define PLRS_IMU_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Constants and definitions.
 */

#define PLRS_IMU_PROTOCOL_VERSION 1
#define PLRS_IMU_DELIMITER 0x00u

#define PLRS_IMU_MAX_PAYLOAD 64
#define PLRS_IMU_HEADER_BYTES 3 // ver, msg_id, seq
#define PLRS_IMU_CRC_BYTES 2
#define PLRS_IMU_MAX_DATA                                                      \
    (PLRS_IMU_HEADER_BYTES + PLRS_IMU_MAX_PAYLOAD + PLRS_IMU_CRC_BYTES)
// COBS adds at least one code byte per frame, plus the trailing delimiter.
#define PLRS_IMU_MAX_FRAME (PLRS_IMU_MAX_DATA + PLRS_IMU_MAX_DATA / 254 + 2)

/**
 * Message IDs. Each selects a payload layout (see docs/rudder_link.md).
 */
typedef enum {
    PLRS_IMU_MSG_HEADING = 0x01,
    PLRS_IMU_MSG_ATTITUDE = 0x02,
} PlrsImuMsgId;

/**
 * Outcome of feeding a byte into the parser: the link's reject reasons plus the
 * "still arriving" and "frame complete" cases.
 */
typedef enum {
    PLRS_IMU_FRAME_NONE = 0,    // no completed frame yet
    PLRS_IMU_FRAME_OK,          // a frame completed and passed every check
    PLRS_IMU_ERR_MALFORMED,     // COBS decode failed, or shorter than header+CRC
    PLRS_IMU_ERR_WRONG_VERSION, // ver byte does not match the protocol version
    PLRS_IMU_ERR_BAD_CRC,       // CRC mismatch
} PlrsImuStatus;

/*
 * Integrity.
 */

#define PLRS_IMU_CRC16_POLY 0x1021u
#define PLRS_IMU_CRC16_INIT 0xFFFFu

/**
 * @brief CRC-16/CCITT-FALSE over a byte buffer.
 *
 * Poly 0x1021, init 0xFFFF, no input/output reflection, no final xor.
 *
 * @param data Bytes to checksum.
 * @param len  Number of bytes.
 *
 * @return The 16-bit CRC.
 */
uint16_t plrs_imu_crc16(const uint8_t *data, size_t len);

/*
 * Framing.
 */

/**
 * @brief COBS-decode a frame block whose trailing delimiter has been stripped.
 *
 * @param block   The encoded block, without PLRS_IMU_DELIMITER.
 * @param len     Number of bytes in the block.
 * @param out     Destination for the decoded bytes.
 * @param out_cap Capacity of out.
 * @param out_len Set to the number of decoded bytes on success.
 *
 * @return true on success; false if the block is malformed or decodes to more
 *   than out_cap bytes.
 */
bool plrs_imu_cobs_decode(const uint8_t *block, size_t len, uint8_t *out,
                          size_t out_cap, size_t *out_len);

/*
 * Messages.
 */

/**
 * @brief Parse a Heading payload: a little-endian float32, compass degrees.
 *
 * @param payload Payload bytes.
 * @param len     Payload length.
 * @param deg_out Set to the heading in degrees on success.
 *
 * @return true on success; false if len is not the size of a float32.
 */
bool plrs_imu_heading_from_payload(const uint8_t *payload, size_t len,
                                   float *deg_out);

/**
 * Attitude payload: heading, roll, pitch (degrees) and yaw rate (deg/s).
 */
typedef struct {
    float heading_deg;
    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
} PlrsImuAttitude;

/**
 * @brief Parse an Attitude payload: four little-endian float32s.
 *
 * @param payload Payload bytes.
 * @param len     Payload length.
 * @param out     Set to the decoded attitude on success.
 *
 * @return true on success; false if len is not four float32s.
 */
bool plrs_imu_attitude_from_payload(const uint8_t *payload, size_t len,
                                    PlrsImuAttitude *out);

/*
 * Receiving.
 */

// Drop a partial frame whose delimiter has not arrived within this long.
#define PLRS_IMU_FRAME_TIMEOUT_MS 50

/**
 * A parsed frame. payload borrows the parser's buffer, so it is valid only
 * until the next plrs_imu_parser_feed.
 */
typedef struct {
    uint8_t msg_id;
    uint8_t seq;
    const uint8_t *payload;
    size_t payload_len;
} PlrsImuFrame;

/**
 * Accumulates bytes up to the COBS delimiter, then decodes, version-checks, and
 * CRC-checks one frame.
 */
typedef struct {
    uint32_t last_advance_ms;
    size_t len;
    bool overflow;
    uint8_t block[PLRS_IMU_MAX_FRAME];
    uint8_t decoded[PLRS_IMU_MAX_DATA];
} PlrsImuParser;

/**
 * @brief Reset the parser and drop the current frame.
 *
 * @param self The parser.
 */
void plrs_imu_parser_reset(PlrsImuParser *self);

/**
 * @brief Check whether the parser is mid frame.
 *
 * @param self The parser.
 *
 * @return true if bytes have accumulated since the last delimiter.
 */
bool plrs_imu_parser_mid_frame(const PlrsImuParser *self);

/**
 * @brief Feed one received byte into the parser.
 *
 * @param self   The parser.
 * @param byte   The current byte.
 * @param now_ms A millisecond timestamp (e.g. HAL_GetTick) for the partial
 *   frame timeout.
 * @param out    Filled with the frame when the return value is PLRS_IMU_FRAME_OK.
 *
 * @return PLRS_IMU_FRAME_NONE while a frame is still arriving, PLRS_IMU_FRAME_OK
 *   with out populated when one completes, or the error explaining why a
 *   completed frame was rejected.
 */
PlrsImuStatus plrs_imu_parser_feed(PlrsImuParser *self, uint8_t byte,
                                   uint32_t now_ms, PlrsImuFrame *out);

#endif /* PLRS_IMU_PROTOCOL_H_ */
