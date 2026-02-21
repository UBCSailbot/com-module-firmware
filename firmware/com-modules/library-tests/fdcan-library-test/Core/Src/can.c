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
 *           NEW UPDATE: Messages are enqueued in interrupt context and dequeued
 *           in application context to prevent data corruption and race conditions
 *           + library adjusted to only standard filter
 */

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Variables ------------------------------------------------------------------*/
#define CAN_RX_QUEUE_SIZE 100 						/* Arbitrary queue size*/
static CAN_Frame CAN_Rx_Queue[CAN_RX_QUEUE_SIZE];	/* Rx Circular Buffer */
static volatile uint8_t canRxHead = 0;				/* Head index of buffer */
static volatile uint8_t canRxTail = 0;				/* Tail index of buffer */
HAL_StatusTypeDef CanStartStatus; 					/* Status of FDCAN start operation */

/* Static Functions -----------------------------------------------------------*/
static int CAN_DequeueFrame(CAN_Frame *frame);
static void CAN_EnqueueFrame(uint32_t id, uint8_t len, const uint8_t *data);
uint8_t dlc_to_bytes(uint8_t dlc);

/* Functions ------------------------------------------------------------------*/
/**
 * @brief 	Initializes the FDCAN module.
 * @details In order:
 * 			Configures standard ID reception filter to Rx FIFO 0
 * 			Configures extended ID reception filter to Rx FIFO 1
 * 			Configures global filter: Filter all remote frames with STD and EXT ID
 * 									  Reject non matching frames with STD ID and EXT ID
 * 			Starts the FDCAN controller (continuous listening CAN bus)
 * 			Activates Notifications
 * @note    Calls Error_Handler() if any configuration step fails.
 */
void CAN_Init(FDCAN_HandleTypeDef *hfdcan1) {
	/*##-1 Configures FDCAN meta data and controllers*/
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x7FF;
	if (HAL_FDCAN_ConfigFilter(hfdcan1, &sFilterConfig) != HAL_OK)
	{
	Error_Handler();
	}

	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x1111111;
	sFilterConfig.FilterID2 = 0x2222222;
	if (HAL_FDCAN_ConfigFilter(hfdcan1, &sFilterConfig) != HAL_OK)
	{
	Error_Handler();
	}

	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
	  Error_Handler();
	}

	/*##-2 Start FDCAN controller (continuous listening CAN bus) ##############*/
	CanStartStatus = HAL_FDCAN_Start(hfdcan1);
	if (CanStartStatus != HAL_OK)
	{
	Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
	Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
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
 * @param   hfdcan1: Pointer to the FDCAN handle structure.
 * @return 	HAL_StatusTypeDef HAL_OK if successful, !HAL_OK (other HAL status) otherwise.
 */
HAL_StatusTypeDef CAN_Transmit(uint32_t Identifier, uint32_t IdType, uint32_t DataLength, uint8_t* DataBuffer, FDCAN_HandleTypeDef *hfdcan1) {
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = Identifier;
    TxHeader.IdType = IdType;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = DataLength;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &TxHeader, DataBuffer);
}

/**
 * @brief   Dequeues the next received CAN frame.
 * @param   frame: Pointer to a CAN_Frame struct that will be filled with the data.
 * @return  HAL_OK if a frame was dequeued, HAL_ERROR if the queue is empty.
 *
 * @note    Called from application context (not ISR).
 */
HAL_StatusTypeDef CAN_Receive(CAN_Frame *frame) {
    if (CAN_DequeueFrame(frame) == 0) return HAL_ERROR;
    return HAL_OK;
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
        uint8_t tmp[64];
        memset(tmp, 0, 64);

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, tmp) != HAL_OK) {
            Error_Handler();
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
        }

		CAN_EnqueueFrame(RxHeader.Identifier, dlc_to_bytes(RxHeader.DataLength), tmp);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    }
    /* added for debug */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
}

/* HELPER FUNCTIONS BELOW */

/**
 * @brief   Adds a received CAN frame to the circular queue.
 * @param   id: CAN message identifier (11-bit standard).
 * @param   len: Length of the data payload in bytes.
 * @param   data: Pointer to the data payload buffer.
 * @details This function implements a circular buffer with overflow handling.
 *          If the queue is full (head catches up to tail), the oldest message is
 *          discarded to make room for the new message; new messages always available.
 */
static void CAN_EnqueueFrame(uint32_t id, uint8_t len, const uint8_t *data) {
    uint8_t next = (canRxHead + 1) % CAN_RX_QUEUE_SIZE;
    if (next == canRxTail)  canRxTail = (canRxTail + 1) % CAN_RX_QUEUE_SIZE;

    CAN_Rx_Queue[canRxHead].RxData1_Identifier = id;
    CAN_Rx_Queue[canRxHead].RxData1_BufferLength = len;
    memcpy(CAN_Rx_Queue[canRxHead].RxData1, data, len);
    canRxHead = next;
}

/**
 * @brief   Removes and retrieves the oldest CAN frame from the circular queue.
 * @param   frame: Pointer to CAN_Frame structure where the retrieved frame will be stored.
 * @return  int: 1 if a frame was successfully retrieved, 0 if the queue is empty.
 * @details This function provides thread-safe reading from the queue by checking if
 *          the queue is empty (tail == head) before attempting to read. The volatile
 *          queue indices ensure memory consistency between ISR and main loop contexts.
 * @note    This function should be called from the main application context, not from ISR.
 *          Always check the return value before using the retrieved frame data.
 */
static int CAN_DequeueFrame(CAN_Frame *frame) {
	if (canRxTail == canRxHead) return 0;
	*frame = CAN_Rx_Queue[canRxTail];
	canRxTail = (canRxTail + 1) % CAN_RX_QUEUE_SIZE;
	return 1;
}

/* DLC to bytes lookup */
uint8_t dlc_to_bytes(uint8_t dlc) {
    static const uint8_t dlc_lut[16] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
    };
    return dlc_lut[dlc & 0x0F];
}
