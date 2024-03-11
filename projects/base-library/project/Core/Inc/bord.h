/**
 * Description: For things that are directly related to the pins on the board.
 *
 * Author: Jordan, Matthew, Peter
 *
 * Note: None
 * */

#ifndef INC_BORD_H_
#define INC_BORD_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"

/* Variables ------------------------------------------------------------------*/
/* Protocols */
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern HAL_StatusTypeDef status;


/* Non-protocols */
extern ADC_HandleTypeDef hadc1;
extern DMA_NodeTypeDef Node_GPDMA1_Channel4;
extern DMA_QListTypeDef List_GPDMA1_Channel4;
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
extern DMA_QListTypeDef pQueueLinkList;
extern DMA_NodeTypeDef Node_GPDMA1_Channel1;
extern DMA_QListTypeDef List_GPDMA1_Channel1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
extern uint8_t UART1_rxBuffer[1];

/* Function prototypes ------------------------------------------------------------------*/
void delay(uint16_t time);
GPIO_PinState gpio_rd_e2();
void gpio_wr_e3();
void gpio_rs_e3();
GPIO_PinState gpio_rd_e4();
void gpio_wr_e5();
void gpio_rs_e5();
void pwm1_init_ch1(uint16_t dutycycle);
void pwm1_set_ch1(uint16_t dutycycle);
void pwm3_init_ch1(uint16_t dutycycle);
void pwm3_set_ch1(uint16_t dutycycle);
extern HAL_StatusTypeDef i2c_wr(I2C_HandleTypeDef handle, uint8_t device_address, uint8_t register_address, uint16_t value);
extern HAL_StatusTypeDef i2c_rd(I2C_HandleTypeDef handle, uint8_t device_address, uint8_t register_address, uint16_t* value);
void uart_rd(UART_HandleTypeDef handle, uint8_t* buffer, int size);
#endif /* INC_BORD_H_ */
