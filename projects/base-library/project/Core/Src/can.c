/*
 * can.c
 *
 *  Created on: Mar 8, 2025
 *      Author: Alisha
 *
 *  @brief 	 This file implements the function prototypes in can.h to initialize and handle FDCAN on an STM32.
 *
 *  @details The implementation includes buffer management, initialization, transmission, and
 *           reception handling with callback functions for handling received messages.
 */

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "main.h"
#include <stdio.h>

/* Variables ------------------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;  		/* Handle for FDCAN1 */
HAL_StatusTypeDef CanStartStatus; 	/* Status of FDCAN start operation */
uint8_t* RxData1 = NULL; 			/* Pointer to receive buffer for FIFO0 (Standard ID)*/
uint8_t* RxData2 = NULL; 			/* Pointer to receive buffer for FIFO1 (Extended ID)*/
uint16_t RxData1_BufferLength = 0; 	/* Length of data received in FIFO0 */
uint16_t RxData2_BufferLength = 0; 	/* Length of data received in FIFO1 */

/* Functions ------------------------------------------------------------------*/

/**
 * @brief Allocates memory for FDCAN receive buffers.
 * @param RxData1_Length: Length of buffer for FIFO0.
 * @param RxData2_Length: Length of buffer for FIFO1.
 */
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

/**
 * @brief 	Initializes the FDCAN module.
 * @details In order:
 * 			Configures standard ID reception filter to Rx buffer
 * 			Configures extended ID reception filter to Rx FIFO 1
 * 			Configures global filter: Filter all remote frames with STD and EXT ID
 * 									  Reject non matching frames with STD ID and EXT ID
 * 			Starts the FDCAN controller (continuous listening CAN bus)
 * 			Activates Notifications
 */
void CAN_Init(void) {
	/*##-1 Configures FDCAN meta data and controllers*/
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

/**
 * @brief 	Transmits a FDCAN message.
 * @param 	Identifier: Message identifier (In-line with Filter used).
 * @param 	IdType: Identifier type (FDCAN_STANDARD_ID or FDCAN_EXTENDED_ID).
 * @param 	DataLength: Length of the TxData buffer in the form of FDCAN_data_length_code
 * 						FDCAN_data_length_code (X bytes data field):
 * 						FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
 * 						FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
 * 						FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
 * 						FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_20,
 * 						FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_48,
 * 						FDCAN_DLC_BYTES_64
 * @param 	DataBuffer: Pointer to the TxData buffer.
 * @return 	HAL_StatusTypeDef HAL_OK if successful, !HAL_OK otherwise.
 */
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

/**
 * @brief 	FDCAN has multiple Callbacks. This section highlights the order of the callbacks
 * 			which you are able to overwrite for specific purposes (like flashing an LED
 * 			for debugging or changing the Tx data in any way).
 * 			These can be overwritten from stm32u5xx_hal_fdcan.h
 * @details When CAN_Transmit is called:
 * 			1. HAL_FDCAN_TxEventFifoCallback(): Triggered when a new entry is added to the Tx Event FIFO after a successful transmission.
 *               - Log transmission status, toggle an LED, or trigger another action upon successful transmission.
 * 			2. HAL_FDCAN_TxBufferCompleteCallback(): Triggered when a dedicated Tx buffer completes transmission.
 *               - This can be used to add functionality after the TxData has been fired.
 * 			3. HAL_FDCAN_RxFifo0Callback(): Triggered when a new message is received in Rx FIFO 0.
 * 			4. HAL_FDCAN_RxFifo1Callback(): Triggered when a new message is received in Rx FIFO 1.
 *               - Process received data, used for debugging (toggle an LED) or trigger other operations based on received messages.
 * 			5. HAL_FDCAN_AbortTxRequest(): Called if transmission is aborted before completion (can be triggered by user).
 *               - Handle retransmission, alert on transmission failure or manually abort for testing.
 * 			6. HAL_FDCAN_HighPriorityMessageCallback(): Triggered when a high-priority message or remote frame is received.
 *               - When processing of urgent messages
 */


/**
 * @brief Callback function for handling messages received in FIFO0.
 * @param hfdcan: Pointer to FDCAN handle.
 * @param RxFifo0ITs: FIFO0 interrupt flags.
 */
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
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
 * @brief Callback function for handling messages received in FIFO1.
 * @param hfdcan: Pointer to FDCAN handle.
 * @param RxFifo1ITs: FIFO1 interrupt flags.
 */
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

/**
 * @brief  	Prints the received RxDataX recived from FIFO0 and FIFO1.
 *
 * @details This function checks if there is received data in the Rx buffers
 * 			If data is available, it prints it in hexadecimal format;
 * 			otherwise, it indicates that no data has been received.
 *
 * @note 	This function assumes that RxData1 and RxData2 have been allocated and
 *       	populated by the Rx FIFO callbacks.
 *
 * @retval 	None
 */
void CAN_PrintRxData(void) {
    if (RxData1 != NULL && RxData1_BufferLength > 0) {
        printf("FIFO0 Received: ");
        for (uint16_t i = 0; i < RxData1_BufferLength; i++) {
            printf("%02X ", RxData1[i]);
        }
        printf("\n");
    } else {
        printf("FIFO0: No data received.\n");
    }

    if (RxData2 != NULL && RxData2_BufferLength > 0) {
        printf("FIFO1 Received: ");
        for (uint16_t i = 0; i < RxData2_BufferLength; i++) {
            printf("%02X ", RxData2[i]);
        }
        printf("\n");
    } else {
        printf("FIFO1: No data received.\n");
    }
}
