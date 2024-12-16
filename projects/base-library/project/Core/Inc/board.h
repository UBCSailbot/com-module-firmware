/*
* board.h
*
* Description: Provides declarations for variables and function prototypes related to CAN, I2C, ADC, PWM, UART communication, and other hardware peripherals.
*
* Created on: Mar 29, 2024
* Author: Peter, Jordan, Katherine
*/

#ifndef INC_BOARD_H_
#define INC_BOARD_H_

/* Standard Library Includes */
#include "main.h"
#include "stdio.h"

/* Hardware Protocol Handles */
extern FDCAN_HandleTypeDef hfdcan1;     /**< FDCAN1 communication handle */
extern I2C_HandleTypeDef hi2c1;          /**< I2C1 communication handle */
extern I2C_HandleTypeDef hi2c2;          /**< I2C2 communication handle */
extern I2C_HandleTypeDef hi2c3;          /**< I2C3 communication handle */
extern I2C_HandleTypeDef hi2c4;          /**< I2C4 communication handle */
extern TIM_HandleTypeDef htim1;          /**< Timer1 handle */
extern TIM_HandleTypeDef htim3;          /**< Timer3 handle */
extern UART_HandleTypeDef huart1;        /**< UART1 communication handle */
extern UART_HandleTypeDef huart2;        /**< UART2 communication handle */
extern HAL_StatusTypeDef status;         /**< Global status variable */

/* Hardware Support Handles */
extern ADC_HandleTypeDef hadc1;          /**< ADC1 handle */
extern DMA_NodeTypeDef Node_GPDMA1_Channel4;     /**< DMA Node for Channel 4 */
extern DMA_QListTypeDef List_GPDMA1_Channel4;    /**< DMA Queue List for Channel 4 */
extern DMA_HandleTypeDef handle_GPDMA1_Channel4; /**< DMA Handle for Channel 4 */
extern DMA_QListTypeDef pQueueLinkList;          /**< Primary Queue Link List */
extern DMA_NodeTypeDef Node_GPDMA1_Channel1;     /**< DMA Node for Channel 1 */
extern DMA_QListTypeDef List_GPDMA1_Channel1;    /**< DMA Queue List for Channel 1 */
extern DMA_HandleTypeDef handle_GPDMA1_Channel1; /**< DMA Handle for Channel 1 */
extern uint8_t UART1_rxBuffer[1];        /**< UART1 Receive Buffer */

/**
 * @brief Blocking delay function
 * @param time Delay duration in milliseconds
 */
void delay(uint16_t time);

/* GPIO Operations */
/**
 * @brief Read GPIO pin state for PE2
 * @return Current pin state
 */
GPIO_PinState gpio_rd_e2(void);

/**
 * @brief Read GPIO pin state for PE4
 * @return Current pin state
 */
GPIO_PinState gpio_rd_e4(void);

/**
 * @brief Set PE3 GPIO pin to HIGH
 */
void gpio_wr_e3(void);

/**
 * @brief Reset PE3 GPIO pin to LOW
 */
void gpio_rs_e3(void);

/**
 * @brief Set PE5 GPIO pin to HIGH
 */
void gpio_wr_e5(void);

/**
 * @brief Reset PE5 GPIO pin to LOW
 */
void gpio_rs_e5(void);

/* PWM Operations */
/**
 * @brief Initialize PWM on Timer1 Channel 1
 * @param dutycycle Initial duty cycle value
 */
void pwm1_init_ch1(uint16_t dutycycle);

/**
 * @brief Set duty cycle for Timer1 Channel 1
 * @param dutycycle New duty cycle value
 */
void pwm1_set_ch1(uint16_t dutycycle);

/**
 * @brief Initialize PWM on Timer3 Channel 1
 * @param dutycycle Initial duty cycle value
 */
void pwm3_init_ch1(uint16_t dutycycle);

/**
 * @brief Set duty cycle for Timer3 Channel 1
 * @param dutycycle New duty cycle value
 */
void pwm3_set_ch1(uint16_t dutycycle);

/* I2C Communication */
/**
 * @brief Read data from I2C device
 * @param handle I2C handle
 * @param device_address Device I2C address
 * @param register_address Register to read from
 * @param value Pointer to store read value
 * @return HAL status of the read operation
 */
HAL_StatusTypeDef i2c_rd(
    I2C_HandleTypeDef handle,
    uint8_t device_address,
    uint8_t register_address,
    uint16_t* value
);

/**
 * @brief Write data to I2C device
 * @param handle I2C handle
 * @param device_address Device I2C address
 * @param register_address Register to write to
 * @param value Value to write
 * @return HAL status of the write operation
 */
HAL_StatusTypeDef i2c_wr(
    I2C_HandleTypeDef handle,
    uint8_t device_address,
    uint8_t register_address,
    uint16_t value
);

/**
 * @brief Read data via UART
 * @param handle UART handle
 * @param buffer Buffer to store received data
 * @param size Number of bytes to read
 */
void uart_rd(UART_HandleTypeDef handle, uint8_t* buffer, int size);

#endif /* INC_BOARD_H_ */
