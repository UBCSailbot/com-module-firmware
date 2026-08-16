#ifndef STM32U5XX_HAL_FDCAN_H
#define STM32U5XX_HAL_FDCAN_H

#include "stm32u5xx_hal.h"

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
} FDCAN_TxHeaderTypeDef;

typedef struct {
    uint32_t Identifier;
    uint32_t DataLength;
} FDCAN_RxHeaderTypeDef;

typedef struct {
    uint32_t IdType;
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
} FDCAN_FilterTypeDef;

typedef struct {
    uint32_t dummy;
} FDCAN_HandleTypeDef;

#define FDCAN_STANDARD_ID 0U
#define FDCAN_EXTENDED_ID 1U

#define FDCAN_FILTER_RANGE 0U
#define FDCAN_FILTER_RANGE_NO_EIDM 1U

#define FDCAN_FILTER_TO_RXFIFO0 0U
#define FDCAN_FILTER_TO_RXFIFO1 1U

#define FDCAN_ACCEPT_IN_RX_FIFO0 0U
#define FDCAN_FILTER_REMOTE 0U

#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 0x01U
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 0x02U

#define FDCAN_RX_FIFO0 0U

#define FDCAN_DATA_FRAME 0U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_ON 1U
#define FDCAN_FD_CAN 1U
#define FDCAN_STORE_TX_EVENTS 1U

#define FDCAN_DLC_BYTES_0 0U
#define FDCAN_DLC_BYTES_1 1U
#define FDCAN_DLC_BYTES_2 2U
#define FDCAN_DLC_BYTES_3 3U
#define FDCAN_DLC_BYTES_4 4U
#define FDCAN_DLC_BYTES_5 5U
#define FDCAN_DLC_BYTES_6 6U
#define FDCAN_DLC_BYTES_7 7U
#define FDCAN_DLC_BYTES_8 8U
#define FDCAN_DLC_BYTES_12 12U
#define FDCAN_DLC_BYTES_16 16U
#define FDCAN_DLC_BYTES_20 20U
#define FDCAN_DLC_BYTES_24 24U
#define FDCAN_DLC_BYTES_32 32U
#define FDCAN_DLC_BYTES_48 48U
#define FDCAN_DLC_BYTES_64 64U

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan,
                                         FDCAN_FilterTypeDef *sFilterConfig);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan,
                                               uint32_t NonMatchingStd,
                                               uint32_t NonMatchingExt,
                                               uint32_t RejectRemoteStd,
                                               uint32_t RejectRemoteExt);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan,
                                                 uint32_t ActiveITs,
                                                 uint32_t BufferIndexes);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                                FDCAN_TxHeaderTypeDef *pTxHeader,
                                                uint8_t *pTxData);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
                                         uint32_t RxFifo,
                                         FDCAN_RxHeaderTypeDef *pRxHeader,
                                         uint8_t *pRxData);

#endif
