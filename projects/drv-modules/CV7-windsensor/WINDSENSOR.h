/*
 * WIND_SENSOR.h
 *
 * Created: March 20, 2025
 * Author: Faaiq Majeed
 * Organization: UBC Sailbot
 *
 */

#ifndef WIND_SENSOR_H
#define WIND_SENSOR_H

#include "stm32u5xx_hal.h"  // Include HAL library for STM32U5 series

//WIND SENTENCE BUFFER SIZE
#define WIND_BUFFER_SIZE 32

//Global Variables for processing wind sentence data
extern uint16_t rx_buffer[1] = {0};
extern uint16_t wind_sentence[WIND_BUFFER_SIZE]; //max sentence identifier length is 10 "PLCJEA870" + \0
extern int wind_sentence_index = 0;
extern uint16_t wind_speed[5]; //in knots
extern uint16_t wind_direction[4]; //in degrees
extern uint16_t wind_temperature[5]; //in Degrees Celsius
extern uint16_t wind_data_can_output[32]; //OUTPUT TO CANBUS


//Function Handles

//Function for processing wind sentence data
void processWindSensorData(uint8_t *input_data);
//Function for handling DMA on USART2 for the wind sensor
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//Function for converting ASCII to INT
int ASCIItoINT(uint8_t input);
#endif

