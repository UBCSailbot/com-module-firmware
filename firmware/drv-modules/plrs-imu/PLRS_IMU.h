/**
 * PLRS-IMU rudder link driver. STM32U5 HAL transport, circular DMA receive.
 *
 * Configure the UART for 115200 8N1 with a circular DMA RX request (see
 * readme.md). Call PLRS_IMU__init once, then PLRS_IMU__service each loop.
 */

#ifndef PLRS_IMU_H_
#define PLRS_IMU_H_

#include "PLRS_IMU_PROTOCOL.h"
#include "stm32u5xx_hal.h"

#define PLRS_IMU_RX_BUFFER_SIZE 256

/**
 * Which attitude source feeds the rudder control model.
 *
 * FUSED is the sender's EKF Attitude (msg 0x02); RAW is the MTi-3 onboard
 * orientation (msg 0x03), which bypasses that filter. Both frames always
 * arrive; this only selects which one getHeel/getYawRate/isAttitudeFresh read.
 */
typedef enum {
    PLRS_IMU_SRC_FUSED = 0,
    PLRS_IMU_SRC_RAW,
} PlrsImuAttitudeSource;

/**
 * Compile-time attitude source. Set to PLRS_IMU_SRC_RAW to steer on the raw MTi
 * heel and yaw rate while the sender's EKF is being worked on. The getters
 * branch on this constant, so main.c needs no change to switch.
 */
#ifndef PLRS_IMU_ATTITUDE_SOURCE
#define PLRS_IMU_ATTITUDE_SOURCE PLRS_IMU_SRC_FUSED
#endif

/**
 * Driver instance. Caller-owned storage; treat the fields as opaque.
 */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_dma[PLRS_IMU_RX_BUFFER_SIZE];
    size_t rx_read;
    PlrsImuParser parser;

    bool has_heading;
    float heading_deg;
    uint32_t heading_ms;

    bool has_attitude;
    float heel_deg;
    float yaw_rate_dps;
    uint32_t attitude_ms;

    uint32_t last_rx_ms;

    bool has_seq;
    uint8_t last_seq;
    uint32_t drops;

    // Raw attitude (msg 0x03): MTi-3 onboard heel and yaw rate, bypassing the
    // sender's EKF. Placed after drops so the imu_live.py word-offset map
    // (heading_deg..drops) stays valid.
    bool has_raw_attitude;
    float raw_heel_deg;
    float raw_yaw_rate_dps;
    uint32_t raw_attitude_ms;

    // Kept last so the fixed imu_live.py word-offset map (heading_deg..drops)
    // stays valid. True when the sender's fused heading is GNSS-anchored.
    bool heading_valid;
} PLRS_IMU;

/**
 * @brief Initialise the driver and start circular DMA reception.
 *
 * @param self  The driver instance.
 * @param huart The UART the IMU is wired to.
 */
void PLRS_IMU__init(PLRS_IMU *self, UART_HandleTypeDef *huart);

/**
 * @brief Drain received bytes and parse any completed frames.
 *
 * @param self The driver instance.
 */
void PLRS_IMU__service(PLRS_IMU *self);

/**
 * @brief Read the latest heading in degrees.
 *
 * @param self    The driver instance.
 * @param deg_out Set to the latest heading.
 *
 * @return true if a heading has been received since init.
 */
bool PLRS_IMU__getHeading(PLRS_IMU *self, float *deg_out);

/**
 * @brief Check whether the latest heading is no older than timeout_ms.
 *
 * @param self       The driver instance.
 * @param timeout_ms Maximum heading age.
 *
 * @return true if a heading has arrived within timeout_ms.
 */
bool PLRS_IMU__isFresh(const PLRS_IMU *self, uint32_t timeout_ms);

/**
 * @brief Read the latest heel angle in degrees (roll, starboard-down positive).
 *
 * Reads the source selected by PLRS_IMU_ATTITUDE_SOURCE (fused or raw).
 *
 * @param self    The driver instance.
 * @param deg_out Set to the latest heel angle.
 *
 * @return true if a frame of the selected source has been received since init.
 */
bool PLRS_IMU__getHeel(PLRS_IMU *self, float *deg_out);

/**
 * @brief Read the latest yaw rate in degrees per second.
 *
 * Reads the source selected by PLRS_IMU_ATTITUDE_SOURCE (fused or raw).
 *
 * @param self     The driver instance.
 * @param dps_out  Set to the latest yaw rate.
 *
 * @return true if a frame of the selected source has been received since init.
 */
bool PLRS_IMU__getYawRate(PLRS_IMU *self, float *dps_out);

/**
 * @brief Check whether the latest attitude (heel, yaw rate) is no older than
 *   timeout_ms.
 *
 * Checks the source selected by PLRS_IMU_ATTITUDE_SOURCE (fused or raw).
 *
 * @param self       The driver instance.
 * @param timeout_ms Maximum attitude age.
 *
 * @return true if a frame of the selected source arrived within timeout_ms.
 */
bool PLRS_IMU__isAttitudeFresh(const PLRS_IMU *self, uint32_t timeout_ms);

/**
 * @brief Whether the latest Attitude frame reported a GNSS-anchored heading.
 *
 * When false, heading is free-drifting (e.g. no dual-antenna GNSS fix) and
 * should not be steered to. Independent of freshness; check
 * PLRS_IMU__isAttitudeFresh too.
 *
 * @param self The driver instance.
 *
 * @return true if the last Attitude frame set heading_valid; false if none
 *   received or the sender flagged heading invalid.
 */
bool PLRS_IMU__isHeadingValid(const PLRS_IMU *self);

/**
 * @brief Tick of the last good frame of any message type.
 *
 * @param self The driver instance.
 *
 * @return The HAL_GetTick value at the last frame, or 0 if none.
 */
uint32_t PLRS_IMU__lastFrameTick(const PLRS_IMU *self);

/**
 * @brief Frames missed, counted from gaps in the sequence number.
 *
 * @param self The driver instance.
 *
 * @return The accumulated drop count.
 */
uint32_t PLRS_IMU__drops(const PLRS_IMU *self);

#endif /* PLRS_IMU_H_ */
