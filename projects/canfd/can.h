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

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_fdcan.h"
#include <stdint.h>
#include <stdlib.h>

/* Variables ------------------------------------------------------------------*/
extern uint8_t* RxData1; 			/* Pointer to receive buffer for FIFO0 (Standard ID)*/
extern uint8_t* RxData2; 			/* Pointer to receive buffer for FIFO1 (Extended ID)*/
extern uint16_t RxData1_Length; 	/* Length of data received in FIFO0 */
extern uint16_t RxData2_Length; 	/* Length of data received in FIFO1 */

/* Function prototypes ------------------------------------------------------------------*/
void CAN_SetRxBufferSize(uint16_t RxData1_Length, uint16_t RxData2_Length);
void CAN_Init(FDCAN_HandleTypeDef *hfdcan1);
HAL_StatusTypeDef CAN_Transmit(uint32_t Identifier, uint32_t IdType, uint32_t DataLength, uint8_t* DataBuffer, FDCAN_HandleTypeDef *hfdcan1);
uint8_t* CAN_Receive(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);
void CAN_PrintRxData(void);
uint8_t dlc_to_bytes(uint8_t dlc);

#endif /* SRC_CAN_H_ */
