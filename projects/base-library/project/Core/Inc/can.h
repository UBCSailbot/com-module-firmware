#ifndef SRC_CAN_H_
#define SRC_CAN_H_

#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_fdcan.h"
#include <stdint.h>
#include <stdlib.h>

extern FDCAN_HandleTypeDef hfdcan1;

void CAN_SetRxBufferSize(uint16_t RxData1_Length, uint16_t RxData2_Length);
void CAN_Init(void);

HAL_StatusTypeDef CAN_Transmit(uint32_t Identifier, uint32_t IdType, uint32_t DataLength, uint8_t* DataBuffer);

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);

extern uint8_t* RxData1;
extern uint8_t* RxData2;
extern uint16_t RxData1_Length;
extern uint16_t RxData2_Length;

#endif /* SRC_CAN_H_ */
