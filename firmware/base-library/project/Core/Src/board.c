/*
 * 	board.c
 *
 * 	Description: Provides basic functions for CAN, I2C, ADC, PWM, and UART communication.
 *
 *  Created on: Mar 29, 2024
 *	Author: Peter, Jordan, Katherine
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"

/* Variables ------------------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
FDCAN_HandleTypeDef hfdcan1;
DMA_NodeTypeDef Node_GPDMA1_Channel4;
DMA_QListTypeDef List_GPDMA1_Channel4;
DMA_HandleTypeDef handle_GPDMA1_Channel4;
DMA_QListTypeDef pQueueLinkList;
DMA_NodeTypeDef Node_GPDMA1_Channel1;
DMA_QListTypeDef List_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
HAL_StatusTypeDef status;

/* Functions ------------------------------------------------------------------*/
/* Print */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)ptr, len);
	return len;
}

/* Delay */
void delay(uint16_t time) {
	HAL_Delay(time);
}

/* ADC */

/* Pulse Width Modulation */
void pwm1_init_ch1(uint16_t dutycycle) {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	htim1.Instance -> CCR1 = dutycycle;
}
void pwm1_set_ch1(uint16_t dutycycle) {
	htim1.Instance -> CCR1 = dutycycle;
}

void pwm3_init_ch1(uint16_t dutycycle) {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	htim3.Instance -> CCR1 = dutycycle;
}
void pwm3_set_ch1(uint16_t dutycycle) {
	htim3.Instance -> CCR1 = dutycycle;
}

/* General-purpose input/output */
GPIO_PinState gpio_rd_e2() {
	return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
}

void gpio_wr_e3() {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}
void gpio_rs_e3() {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

GPIO_PinState gpio_rd_e4() {
	return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
}

void gpio_wr_e5() {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}
void gpio_rs_e5() {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
}

/* I2C */
HAL_StatusTypeDef i2c_rd(I2C_HandleTypeDef handle, uint8_t device_address, uint8_t register_address, uint16_t* value) {
	status = HAL_I2C_Mem_Read(&handle, device_address<<1, register_address, sizeof(register_address), (uint8_t*)value, sizeof(*value), HAL_MAX_DELAY);
	if (status) {
		return status;
	}

	return status;
}

HAL_StatusTypeDef i2c_wr(I2C_HandleTypeDef handle, uint8_t device_address, uint8_t register_address, uint16_t value) {
	status = HAL_I2C_Mem_Write(&handle, device_address<<1, register_address, sizeof(register_address), (uint8_t*)&value, sizeof(value), HAL_MAX_DELAY);
	if (status) {
		return status;
	}

	return status;
}

/* USART */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	printf("%hu\r\n", *UART1_rxBuffer);
    HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);
}

/* CAN */



