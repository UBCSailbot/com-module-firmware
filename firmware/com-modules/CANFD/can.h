/*
 * can.h
 *
 *  Created on: Mar 8, 2025
 *      Author: Alisha
 *
 *  @brief Header file for FDCAN library.
 *
 *  @details This file contains function prototypes and global variables for configuring,
 *           transmitting, and receiving FDCAN messages on the STM32.
 */
#ifndef SRC_CAN_H_
#define SRC_CAN_H_

/* Includes ----------------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_fdcan.h"
#include <stdint.h>
#include <stdlib.h>

/* CAN_frame struct for Rx buffer (standard filter). Provides Rx ID, length and buffer -*/
typedef struct {
    uint32_t RxData1_Identifier;
    uint8_t  RxData1_BufferLength;
    uint8_t  RxData1[64];
} CAN_Frame;

/* Function prototypes ------------------------------------------------------------------*/
HAL_StatusTypeDef CAN_Init(FDCAN_HandleTypeDef *hfdcan1);
HAL_StatusTypeDef CAN_Transmit(uint32_t Identifier, uint32_t IdType, uint32_t DataLength, uint8_t* DataBuffer, FDCAN_HandleTypeDef *hfdcan1);
HAL_StatusTypeDef CAN_Receive(uint8_t *RxData_buffer);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
#endif /* SRC_CAN_H_ */
