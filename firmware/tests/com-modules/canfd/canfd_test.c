#include "can.h"
#include "test_assert.h"

#include <stdio.h>
#include <string.h>

static int g_failures = 0;

static FDCAN_TxHeaderTypeDef g_last_tx_header;
static uint8_t g_last_tx_data[64];
static uint8_t g_last_tx_data_size = 0;
static int g_last_tx_called = 0;
static HAL_StatusTypeDef g_next_tx_status = HAL_OK;

/**
 * @brief Reset transmit capture state.
 *
 * @param void
 * @return void
 */
static void reset_tx_capture(void) {
  memset(&g_last_tx_header, 0, sizeof(g_last_tx_header));
  memset(g_last_tx_data, 0, sizeof(g_last_tx_data));
  g_last_tx_data_size = 0;
  g_last_tx_called = 0;
  g_next_tx_status = HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan,
                                         FDCAN_FilterTypeDef *sFilterConfig) {
  (void)hfdcan;
  (void)sFilterConfig;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan,
                                               uint32_t NonMatchingStd,
                                               uint32_t NonMatchingExt,
                                               uint32_t RejectRemoteStd,
                                               uint32_t RejectRemoteExt) {
  (void)hfdcan;
  (void)NonMatchingStd;
  (void)NonMatchingExt;
  (void)RejectRemoteStd;
  (void)RejectRemoteExt;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan) {
  (void)hfdcan;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan,
                                                 uint32_t ActiveITs,
                                                 uint32_t BufferIndexes) {
  (void)hfdcan;
  (void)ActiveITs;
  (void)BufferIndexes;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                                FDCAN_TxHeaderTypeDef *pTxHeader,
                                                uint8_t *pTxData) {
  uint8_t size = 0;

  (void)hfdcan;

  g_last_tx_called = 1;
  if (pTxHeader) {
    g_last_tx_header = *pTxHeader;
    size = (uint8_t)pTxHeader->DataLength;
  }

  if (size > sizeof(g_last_tx_data)) {
    size = (uint8_t)sizeof(g_last_tx_data);
  }

  if (pTxData && size > 0) {
    memcpy(g_last_tx_data, pTxData, size);
    g_last_tx_data_size = size;
  }

  return g_next_tx_status;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
                                         uint32_t RxFifo,
                                         FDCAN_RxHeaderTypeDef *pRxHeader,
                                         uint8_t *pRxData) {
  (void)hfdcan;
  (void)RxFifo;
  (void)pRxHeader;
  (void)pRxData;
  return HAL_OK;
}

/**
 * @brief Stub Error_Handler for host tests.
 *
 * @param void
 * @return void
 */
void Error_Handler(void) {
  g_failures++;
}

/**
 * @brief Validate CAN_Transmit builds the expected header and payload.
 *
 * @param void
 * @return void
 */
static void test_can_transmit_sets_header_and_payload(void) {
  FDCAN_HandleTypeDef hfdcan = {0};
  uint8_t payload[4] = {0x2D, 0x00, 0x66, 0x00};

  reset_tx_capture();

  TEST_ASSERT(&g_failures,
              CAN_Transmit(0x040, FDCAN_STANDARD_ID, FDCAN_DLC_BYTES_4,
                           payload, &hfdcan) == HAL_OK);
  TEST_ASSERT(&g_failures, g_last_tx_called == 1);
  TEST_ASSERT(&g_failures, g_last_tx_header.Identifier == 0x040);
  TEST_ASSERT(&g_failures, g_last_tx_header.IdType == FDCAN_STANDARD_ID);
  TEST_ASSERT(&g_failures, g_last_tx_header.TxFrameType == FDCAN_DATA_FRAME);
  TEST_ASSERT(&g_failures, g_last_tx_header.DataLength == FDCAN_DLC_BYTES_4);
  TEST_ASSERT(&g_failures,
              g_last_tx_header.ErrorStateIndicator == FDCAN_ESI_ACTIVE);
  TEST_ASSERT(&g_failures, g_last_tx_header.BitRateSwitch == FDCAN_BRS_ON);
  TEST_ASSERT(&g_failures, g_last_tx_header.FDFormat == FDCAN_FD_CAN);
  TEST_ASSERT(&g_failures,
              g_last_tx_header.TxEventFifoControl == FDCAN_STORE_TX_EVENTS);
  TEST_ASSERT(&g_failures, g_last_tx_data_size == 4);
  TEST_ASSERT(&g_failures, memcmp(g_last_tx_data, payload, 4) == 0);
}

/**
 * @brief Validate CAN_Transmit propagates the HAL return status.
 *
 * @param void
 * @return void
 */
static void test_can_transmit_propagates_status(void) {
  FDCAN_HandleTypeDef hfdcan = {0};
  uint8_t payload[1] = {0xAA};

  reset_tx_capture();
  g_next_tx_status = HAL_ERROR;

  TEST_ASSERT(&g_failures,
              CAN_Transmit(0x041, FDCAN_STANDARD_ID, FDCAN_DLC_BYTES_1,
                           payload, &hfdcan) == HAL_ERROR);
  TEST_ASSERT(&g_failures, g_last_tx_called == 1);
}

/**
 * @brief Test runner entry point.
 *
 * @param void
 * @return 0 when all tests pass, nonzero otherwise.
 */
int main(void) {
  test_can_transmit_sets_header_and_payload();
  test_can_transmit_propagates_status();

  if (g_failures == 0) {
    printf("PASS\n");
    return 0;
  }
  printf("FAILURES: %d\n", g_failures);
  return 1;
}
