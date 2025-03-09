/*
 * can.c
 *
 *  Created on: Mar 8, 2025
 *      Author: Alisha
 *      (No documentation) FDCAN Library
 */
#include "can.h"
#include "main.h"

FDCAN_HandleTypeDef hfdcan1;
HAL_StatusTypeDef CanStartStatus;
uint8_t* RxData1 = NULL;
uint8_t* RxData2 = NULL;
uint16_t RxData1_BufferLength = 0;
uint16_t RxData2_BufferLength = 0;

/* Setting the buffer size for FDCAN,
 * Allocating the size of length passed in and freeing when not null*/
void CAN_SetRxBufferSize(uint16_t RxData1_Length, uint16_t RxData2_Length) {
    if (RxData1 != NULL) {
        free(RxData1);
    }
    if (RxData2 != NULL) {
        free(RxData2);
    }

    RxData1_BufferLength = RxData1_Length;
    RxData2_BufferLength = RxData2_Length;

    RxData1 = (uint8_t* )malloc(RxData1_Length);
    RxData2 = (uint8_t* )malloc(RxData2_Length);
    if (RxData1 == NULL || RxData2 == NULL) {
        Error_Handler();
    }
}

/* Initialize FDCAN
 * Blackboxed: standard and extended filters, global filter and starting the can controller
 * Starting the controller could be another function?
 * void
 * */
void CAN_Init(void) {
	/* Configure standard ID reception filter to Rx buffer 0*/
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x444;
	sFilterConfig.FilterID2 = 0x555;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
	Error_Handler();
	}

	/* Configure extended ID reception filter to Rx FIFO 1 */
	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x1111111;
	sFilterConfig.FilterID2 = 0x2222222;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
	Error_Handler();
	}

	/* Configure global filter:
	 Filter all remote frames with STD and EXT ID
	 Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
	  Error_Handler();
	}

	/*##-2 Start FDCAN controller (continuous listening CAN bus) ##############*/
	CanStartStatus = HAL_FDCAN_Start(&hfdcan1);
	if (CanStartStatus != HAL_OK)
	{
	Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
	Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
	{
	Error_Handler();
	}
}

/* Transmits CAN
 * Takes in metadata as parameters w the Tx buffer
 * Returns: HAL_FDCAN_AddMessageToTxFifoQ
 * */
HAL_StatusTypeDef CAN_Transmit(uint32_t Identifier, uint32_t IdType, uint32_t DataLength, uint8_t* DataBuffer) {
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = Identifier;
    TxHeader.IdType = IdType;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = DataLength;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, DataBuffer);
}

/* Callback */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        FDCAN_RxHeaderTypeDef RxHeader;
        if (RxData1 == NULL) {
            Error_Handler();
        }
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData1) != HAL_OK) {
            Error_Handler();
        }
        //check actual length incoming against the buf len rather than stringcmp?
        RxData1_BufferLength = RxHeader.DataLength;
    }
    /* added for debug */
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        FDCAN_RxHeaderTypeDef RxHeader;
        if (RxData2 == NULL) {
            Error_Handler();
        }
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData2) != HAL_OK) {
            Error_Handler();
        }
        RxData2_BufferLength = RxHeader.DataLength;
    }
}
