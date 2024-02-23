/**
 * Description: For things that are directly related to the pins on the board.
 *
 * Author: Jordan, Matthew, Peter
 *
 * Note: None
 * */

/* Includes ------------------------------------------------------------------*/
#include "bord.h"

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
	HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

/* Delay */
void delay(uint16_t time) {
	HAL_Delay(time);
}

/* ADC */

/* Pulse Width Modulation */
void pwm_init_ch1(uint16_t dutycycle) {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	htim1.Instance -> CCR1 = dutycycle;
}

void pwm_set_ch1(uint16_t dutycycle) {
	htim1.Instance -> CCR1 = dutycycle;
}

void pwm_init_ch3(uint16_t dutycycle) {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	htim1.Instance -> CCR3 = dutycycle;
}

void pwm_set_ch3(uint16_t dutycycle) {
	htim1.Instance -> CCR3 = dutycycle;
}

/* General-purpose input/output */
GPIO_PinState gpio_rd_e2() {return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);}
void gpio_wr_e3() {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);}
void gpio_rs_e3() {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);}
GPIO_PinState gpio_rd_e4() {return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);}
void gpio_wr_e5() {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);}
void gpio_rs_e5() {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);}

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

/*
HAL_StatusTypeDef uart_wr(UART_HandleTypeDef handle, uint8_t msg) {
	status = HAL_UART_Transmit(&handle, msg, sizeof(msg), HAL_MAX_DELAY);
	if (status) {
		return status;
	}

	return status;
}
*/

/* CAN */



