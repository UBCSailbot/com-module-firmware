/*
* board.c
*
* Description: Provides basic functions for CAN, I2C, ADC, PWM, and UART communication.
*
* Created on: Mar 29, 2024
* Author: Peter, Jordan, Katherine
*/

#include "board.h"

/* Global Hardware Handles */
ADC_HandleTypeDef hadc1;
FDCAN_HandleTypeDef hfdcan1;
DMA_NodeTypeDef Node_GPDMA1_Channel4;
DMA_QListTypeDef List_GPDMA1_Channel4;
DMA_HandleTypeDef handle_GPDMA1_Channel4;
DMA_QListTypeDef pQueueLinkList;
DMA_NodeTypeDef Node_GPDMA1_Channel1;
DMA_QListTypeDef List_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
I2C_HandleTypeDef hi2c1, hi2c2, hi2c3, hi2c4;
TIM_HandleTypeDef htim1, htim3;
UART_HandleTypeDef huart1, huart2;
HAL_StatusTypeDef status;

/**
 * @brief Custom write function to redirect printf to UART
 * @param file File descriptor (unused)
 * @param ptr Pointer to data to be written
 * @param len Length of data
 * @return Number of bytes written
 */
int _write(int file, char *ptr, int len) {
    HAL_StatusTypeDef result = HAL_UART_Transmit_IT(&huart1, (uint8_t*)ptr, len);
    return (result == HAL_OK) ? len : 0;
}

/**
 * @brief Blocking delay function
 * @param time Delay duration in milliseconds
 */
void delay(uint16_t time) {
    HAL_Delay(time);
}

/* PWM Operations */
/**
 * @brief Initialize PWM on Timer1 Channel 1
 * @param dutycycle Initial duty cycle value
 */
void pwm1_init_ch1(uint16_t dutycycle) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    htim1.Instance->CCR1 = dutycycle;
}

/**
 * @brief Set duty cycle for Timer1 Channel 1
 * @param dutycycle New duty cycle value
 */
void pwm1_set_ch1(uint16_t dutycycle) {
    htim1.Instance->CCR1 = dutycycle;
}

/**
 * @brief Initialize PWM on Timer3 Channel 1
 * @param dutycycle Initial duty cycle value
 */
void pwm3_init_ch1(uint16_t dutycycle) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    htim3.Instance->CCR1 = dutycycle;
}

/**
 * @brief Set duty cycle for Timer3 Channel 1
 * @param dutycycle New duty cycle value
 */
void pwm3_set_ch1(uint16_t dutycycle) {
    htim3.Instance->CCR1 = dutycycle;
}

/* GPIO Operations */
/**
 * @brief Read GPIO pin state for PE2
 * @return Current pin state
 */
GPIO_PinState gpio_rd_e2(void) {
    return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
}

/**
 * @brief Read GPIO pin state for PE4
 * @return Current pin state
 */
GPIO_PinState gpio_rd_e4(void) {
    return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
}

/**
 * @brief Set PE3 GPIO pin to HIGH
 */
void gpio_wr_e3(void) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

/**
 * @brief Reset PE3 GPIO pin to LOW
 */
void gpio_rs_e3(void) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

/**
 * @brief Set PE5 GPIO pin to HIGH
 */
void gpio_wr_e5(void) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
 * @brief Reset PE5 GPIO pin to LOW
 */
void gpio_rs_e5(void) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
}

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
) {
    status = HAL_I2C_Mem_Read(
        &handle,
        device_address << 1,
        register_address,
        sizeof(register_address),
        (uint8_t*)value,
        sizeof(*value),
        HAL_MAX_DELAY
    );
    return status;
}

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
) {
    status = HAL_I2C_Mem_Write(
        &handle,
        device_address << 1,
        register_address,
        sizeof(register_address),
        (uint8_t*)&value,
        sizeof(value),
        HAL_MAX_DELAY
    );
    return status;
}

/**
 * @brief UART Receive Complete Callback
 * @param huart UART handle that triggered the callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // Print the received byte
    printf("%hu\r\n", *UART1_rxBuffer);

    // Restart interrupt-based reception
    HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);
}

/**
 * @brief Placeholder for UART read function
 * @param handle UART handle
 * @param buffer Buffer to store received data
 * @param size Number of bytes to read
 */
void uart_rd(UART_HandleTypeDef handle, uint8_t* buffer, int size) {
    // Implement UART read logic here
    HAL_UART_Receive(&handle, buffer, size, HAL_MAX_DELAY);
}
