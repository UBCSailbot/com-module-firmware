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
    uint32_t last_rx_ms;

    bool has_seq;
    uint8_t last_seq;
    uint32_t drops;
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
