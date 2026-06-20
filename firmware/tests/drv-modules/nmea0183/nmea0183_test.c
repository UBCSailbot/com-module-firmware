#include "NMEA0183.h"
#include "nmea_test_utils.h"
#include "test_assert.h"

#include <stdio.h>
#include <string.h>

static int g_failures = 0;

/**
 * @brief Load a raw sample line and append CRLF terminators.
 *
 * @param msg Output sentence container.
 * @param line Raw NMEA line without CRLF.
 * @return void
 */
static void load_sentence_line(NMEA0183Raw *msg, const char *line) {
  size_t len = strlen(line);
  memcpy(msg->scentenceData, line, len);
  msg->scentenceData[len++] = '\r';
  msg->scentenceData[len++] = '\n';
  msg->scentenceLength = (uint8_t)len;
}

/**
 * @brief Validate short messages fail termination checks.
 *
 * @param void
 * @return void
 */
static void test_check_message_rejects_short(void) {
  NMEA0183Raw msg = {0};
  msg.scentenceLength = 4;
  TEST_ASSERT(&g_failures,
              NMEA0183__checkMessage(&msg) == BAD_TERMINATION_SEQUENCE);
}

/**
 * @brief Validate bad start characters are rejected.
 *
 * @param void
 * @return void
 */
static void test_check_message_bad_start(void) {
  NMEA0183Raw msg = {0};
  msg.scentenceData[0] = '?';
  msg.scentenceData[1] = '*';
  msg.scentenceData[2] = '0';
  msg.scentenceData[3] = '0';
  msg.scentenceData[4] = '\r';
  msg.scentenceData[5] = '\n';
  msg.scentenceLength = 6;
  TEST_ASSERT(&g_failures, NMEA0183__checkMessage(&msg) == BAD_START_CHARACTER);
}

/**
 * @brief Validate missing CRLF is rejected.
 *
 * @param void
 * @return void
 */
static void test_check_message_bad_termination(void) {
  NMEA0183Raw msg = {0};
  nmea_test_build_sentence(&msg, '$', "IIMWV,045.0,R");
  msg.scentenceData[msg.scentenceLength - 1] = 'X';
  TEST_ASSERT(&g_failures,
              NMEA0183__checkMessage(&msg) == BAD_TERMINATION_SEQUENCE);
}

/**
 * @brief Validate incorrect checksum is rejected.
 *
 * @param void
 * @return void
 */
static void test_check_message_bad_checksum(void) {
  NMEA0183Raw msg = {0};
  nmea_test_build_sentence(&msg, '$', "IIMWV,045.0,R");
  msg.scentenceData[msg.scentenceLength - 3] = '0';
  TEST_ASSERT(&g_failures, NMEA0183__checkMessage(&msg) == BAD_CHECK_SUM);
}

/**
 * @brief Validate a few real sample lines parse correctly.
 *
 * @param void
 * @return void
 */
static void test_check_message_sample_lines(void) {
  NMEA0183Raw msg = {0};

  load_sentence_line(&msg, "!AIVDM,1,1,,A,133sVfPP00PD>hRMDH@jNOvN20S8,0*7F");
  TEST_ASSERT(&g_failures, NMEA0183__checkMessage(&msg) == GOOD_MESSAGE);
  TEST_ASSERT(&g_failures, NMEA0183__getScentenceType(&msg) == MESSAGE_VDM);

  load_sentence_line(&msg,
                     "!AIVDM,2,1,9,B,53nFBv01SJ<thHp6220H4heHTf2222222222221?"
                     "50:454o<`9QSlUDp,0*09");
  TEST_ASSERT(&g_failures, NMEA0183__checkMessage(&msg) == GOOD_MESSAGE);
  TEST_ASSERT(&g_failures, NMEA0183__getScentenceType(&msg) == MESSAGE_VDM);
}

/**
 * @brief Validate field splitting and sentence type hashing.
 *
 * @param void
 * @return void
 */
static void test_good_message_fields_and_type(void) {
  NMEA0183Raw msg = {0};
  nmea_test_build_sentence(&msg, '$', "IIMWV,045.0,R,10.2,N,A");
  TEST_ASSERT(&g_failures, NMEA0183__checkMessage(&msg) == GOOD_MESSAGE);

  TEST_ASSERT(&g_failures,
              strcmp((char *)NMEA0183__getField(&msg, 0), "IIMWV") == 0);
  TEST_ASSERT(&g_failures,
              strcmp((char *)NMEA0183__getField(&msg, 1), "045.0") == 0);
  TEST_ASSERT(&g_failures,
              strcmp((char *)NMEA0183__getField(&msg, 2), "R") == 0);
  TEST_ASSERT(&g_failures, NMEA0183__getField(&msg, 50) == NULL);

  TEST_ASSERT(&g_failures, NMEA0183__getScentenceType(&msg) == MESSAGE_MWV);
}

/**
 * @brief Validate helper functions for buffer indices.
 *
 * @param void
 * @return void
 */
static void test_buffer_helpers(void) {
  NMEA0183 nmea = {0};

  nmea.dataBufferReadIndex = 0;
  nmea.dataBufferWriteIndex = 0;
  TEST_ASSERT(&g_failures, NMEA0183__itemsInBuffer(&nmea) == 0);

  nmea.dataBufferReadIndex = 1;
  nmea.dataBufferWriteIndex = 3;
  TEST_ASSERT(&g_failures, NMEA0183__itemsInBuffer(&nmea) == 2);

  TEST_ASSERT(&g_failures,
              NMEA0183__getTopBufferItem(&nmea) == &nmea.dataBuffer[1]);
  NMEA0183__incrementReadIndex(&nmea);
  TEST_ASSERT(&g_failures, nmea.dataBufferReadIndex == 2);
}

/**
 * @brief Initialize a UART/DMA stub for IRQ tests.
 *
 * @param huart UART handle to initialize.
 * @param uart_instance UART register block.
 * @param dma DMA handle to initialize.
 * @return void
 */
static void setup_uart(UART_HandleTypeDef *huart, USART_TypeDef *uart_instance,
                       DMA_HandleTypeDef *dma) {
  memset(uart_instance, 0, sizeof(*uart_instance));
  memset(dma, 0, sizeof(*dma));
  huart->Instance = uart_instance;
  huart->hdmarx = dma;
}

/**
 * @brief Simulate a CMF IRQ with a given DMA length.
 *
 * @param huart UART handle with DMA state.
 * @param len Received length to report.
 * @return void
 */
static void trigger_irq(UART_HandleTypeDef *huart, uint16_t len) {
  huart->hdmarx->counter = (uint16_t)(MAX_SENTENCE_LENGTH + 1 - len);
  huart->Instance->ISR = USART_ISR_CMF;
  NMEA0183__IRQHandler(huart);
}

/**
 * @brief Copy a sentence into the active DMA buffer.
 *
 * @param nmea NMEA instance with receive buffers.
 * @param msg Sentence to copy.
 * @return void
 */
static void copy_sentence_to_dma(NMEA0183 *nmea, const NMEA0183Raw *msg) {
  memcpy(nmea->receiveBuffers[nmea->receiveBufferPosition], msg->scentenceData,
         msg->scentenceLength);
}

/**
 * @brief Verify overflow handling when the ring buffer wraps.
 *
 * @param void
 * @return void
 */
static void test_irq_buffer_overflow(void) {
  UART_HandleTypeDef huart = {0};
  USART_TypeDef uart_instance = {0};
  DMA_HandleTypeDef dma = {0};
  setup_uart(&huart, &uart_instance, &dma);

  NMEA0183 *nmea = NMEA0183__create(&huart);
  TEST_ASSERT(&g_failures, nmea != NULL);

  NMEA0183Raw msg = {0};
  nmea_test_build_sentence(&msg, '$', "IIMWV,045.0,R,10.2,N,A");

  for (int i = 0; i < MAX_DATA_BUFFER_SIZE + 2; i++) {
    copy_sentence_to_dma(nmea, &msg);
    trigger_irq(&huart, msg.scentenceLength);
  }

  TEST_ASSERT(&g_failures, nmea->overflowed == true);
  TEST_ASSERT(&g_failures, nmea->dataBufferReadIndex == 3);
  TEST_ASSERT(&g_failures, nmea->dataBufferWriteIndex == 2);

  NMEA0183__destroy(nmea);
}

/**
 * @brief Verify short DMA receives are ignored.
 *
 * @param void
 * @return void
 */
static void test_irq_ignores_short_receive(void) {
  UART_HandleTypeDef huart = {0};
  USART_TypeDef uart_instance = {0};
  DMA_HandleTypeDef dma = {0};
  setup_uart(&huart, &uart_instance, &dma);

  NMEA0183 *nmea = NMEA0183__create(&huart);
  TEST_ASSERT(&g_failures, nmea != NULL);

  NMEA0183Raw msg = {0};
  nmea_test_build_sentence(&msg, '$', "IIMWV,045.0,R");
  copy_sentence_to_dma(nmea, &msg);
  trigger_irq(&huart, 2);

  TEST_ASSERT(&g_failures, NMEA0183__itemsInBuffer(nmea) == 0);

  NMEA0183__destroy(nmea);
}

/**
 * @brief Stub Error_Handler for host tests.
 *
 * @param void
 * @return void
 */
void Error_Handler(void) { g_failures++; }

/**
 * @brief Test runner entry point.
 *
 * @param void
 * @return 0 when all tests pass, nonzero otherwise.
 */
int main(void) {
  test_check_message_rejects_short();
  test_check_message_bad_start();
  test_check_message_bad_termination();
  test_check_message_bad_checksum();
  test_check_message_sample_lines();
  test_good_message_fields_and_type();
  test_buffer_helpers();
  test_irq_buffer_overflow();
  test_irq_ignores_short_receive();

  if (g_failures == 0) {
    printf("PASS\n");
    return 0;
  }
  printf("FAILURES: %d\n", g_failures);
  return 1;
}
