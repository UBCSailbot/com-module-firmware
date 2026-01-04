#include "NMEA0183.h"

#include <stdio.h>
#include <string.h>

static int g_failures = 0;

static void test_assert(int condition, const char *expr, const char *file, int line) {
    if (!condition) {
        printf("FAIL: %s:%d: %s\n", file, line, expr);
        g_failures++;
    }
}

#define TEST_ASSERT(cond) test_assert((cond), #cond, __FILE__, __LINE__)

static uint8_t to_hex_ascii(uint8_t value) {
    if (value < 10) {
        return (uint8_t)('0' + value);
    }
    return (uint8_t)('A' + (value - 10));
}

static void build_sentence(NMEA0183Raw *msg, char start, const char *body) {
    size_t body_len = strlen(body);
    uint8_t checksum = 0;
    size_t index = 0;

    msg->scentenceData[index++] = (uint8_t)start;
    memcpy(&msg->scentenceData[index], body, body_len);
    for (size_t i = 0; i < body_len; i++) {
        checksum ^= (uint8_t)body[i];
    }
    index += body_len;

    msg->scentenceData[index++] = '*';
    msg->scentenceData[index++] = to_hex_ascii((uint8_t)(checksum >> 4));
    msg->scentenceData[index++] = to_hex_ascii((uint8_t)(checksum & 0x0F));
    msg->scentenceData[index++] = '\r';
    msg->scentenceData[index++] = '\n';
    msg->scentenceLength = (uint8_t)index;
}

static void load_sentence_line(NMEA0183Raw *msg, const char *line) {
    size_t len = strlen(line);
    memcpy(msg->scentenceData, line, len);
    msg->scentenceData[len++] = '\r';
    msg->scentenceData[len++] = '\n';
    msg->scentenceLength = (uint8_t)len;
}

static void test_check_message_rejects_short(void) {
    NMEA0183Raw msg = {0};
    msg.scentenceLength = 4;
    TEST_ASSERT(NMEA0183__checkMessage(&msg) == BAD_TERMINATION_SEQUENCE);
}

static void test_check_message_bad_start(void) {
    NMEA0183Raw msg = {0};
    msg.scentenceData[0] = '?';
    msg.scentenceData[1] = '*';
    msg.scentenceData[2] = '0';
    msg.scentenceData[3] = '0';
    msg.scentenceData[4] = '\r';
    msg.scentenceData[5] = '\n';
    msg.scentenceLength = 6;
    TEST_ASSERT(NMEA0183__checkMessage(&msg) == BAD_START_CHARACTER);
}

static void test_check_message_bad_termination(void) {
    NMEA0183Raw msg = {0};
    build_sentence(&msg, '$', "IIMWV,045.0,R");
    msg.scentenceData[msg.scentenceLength - 1] = 'X';
    TEST_ASSERT(NMEA0183__checkMessage(&msg) == BAD_TERMINATION_SEQUENCE);
}

static void test_check_message_bad_checksum(void) {
    NMEA0183Raw msg = {0};
    build_sentence(&msg, '$', "IIMWV,045.0,R");
    msg.scentenceData[msg.scentenceLength - 3] = '0';
    TEST_ASSERT(NMEA0183__checkMessage(&msg) == BAD_CHECK_SUM);
}

static void test_check_message_sample_lines(void) {
    NMEA0183Raw msg = {0};

    load_sentence_line(&msg, "!AIVDM,1,1,,A,133sVfPP00PD>hRMDH@jNOvN20S8,0*7F");
    TEST_ASSERT(NMEA0183__checkMessage(&msg) == GOOD_MESSAGE);
    TEST_ASSERT(NMEA0183__getScentenceType(&msg) == MESSAGE_VDM);

    load_sentence_line(&msg, "!AIVDM,2,1,9,B,53nFBv01SJ<thHp6220H4heHTf2222222222221?50:454o<`9QSlUDp,0*09");
    TEST_ASSERT(NMEA0183__checkMessage(&msg) == GOOD_MESSAGE);
    TEST_ASSERT(NMEA0183__getScentenceType(&msg) == MESSAGE_VDM);
}

static void test_good_message_fields_and_type(void) {
    NMEA0183Raw msg = {0};
    build_sentence(&msg, '$', "IIMWV,045.0,R,10.2,N,A");
    TEST_ASSERT(NMEA0183__checkMessage(&msg) == GOOD_MESSAGE);

    TEST_ASSERT(strcmp((char *)NMEA0183__getField(&msg, 0), "IIMWV") == 0);
    TEST_ASSERT(strcmp((char *)NMEA0183__getField(&msg, 1), "045.0") == 0);
    TEST_ASSERT(strcmp((char *)NMEA0183__getField(&msg, 2), "R") == 0);
    TEST_ASSERT(NMEA0183__getField(&msg, 50) == NULL);

    TEST_ASSERT(NMEA0183__getScentenceType(&msg) == MESSAGE_MWV);
}

static void test_buffer_helpers(void) {
    NMEA0183 nmea = {0};

    nmea.dataBufferReadIndex = 0;
    nmea.dataBufferWriteIndex = 0;
    TEST_ASSERT(NMEA0183__itemsInBuffer(&nmea) == 0);

    nmea.dataBufferReadIndex = 1;
    nmea.dataBufferWriteIndex = 3;
    TEST_ASSERT(NMEA0183__itemsInBuffer(&nmea) == 2);

    TEST_ASSERT(NMEA0183__getTopBufferItem(&nmea) == &nmea.dataBuffer[1]);
    NMEA0183__incrementReadIndex(&nmea);
    TEST_ASSERT(nmea.dataBufferReadIndex == 2);
}

static void setup_uart(UART_HandleTypeDef *huart, USART_TypeDef *uart_instance,
                       DMA_HandleTypeDef *dma) {
    memset(uart_instance, 0, sizeof(*uart_instance));
    memset(dma, 0, sizeof(*dma));
    huart->Instance = uart_instance;
    huart->hdmarx = dma;
}

static void trigger_irq(UART_HandleTypeDef *huart, uint16_t len) {
    huart->hdmarx->counter = (uint16_t)(MAX_SENTENCE_LENGTH + 1 - len);
    huart->Instance->ISR = USART_ISR_CMF;
    NMEA0183__IRQHandler(huart);
}

static void copy_sentence_to_dma(NMEA0183 *nmea, const NMEA0183Raw *msg) {
    memcpy(nmea->receiveBuffers[nmea->receiveBufferPosition],
           msg->scentenceData,
           msg->scentenceLength);
}

static void test_irq_buffer_overflow(void) {
    UART_HandleTypeDef huart = {0};
    USART_TypeDef uart_instance = {0};
    DMA_HandleTypeDef dma = {0};
    setup_uart(&huart, &uart_instance, &dma);

    NMEA0183 *nmea = NMEA0183__create(&huart);
    TEST_ASSERT(nmea != NULL);

    NMEA0183Raw msg = {0};
    build_sentence(&msg, '$', "IIMWV,045.0,R,10.2,N,A");

    for (int i = 0; i < MAX_DATA_BUFFER_SIZE + 2; i++) {
        copy_sentence_to_dma(nmea, &msg);
        trigger_irq(&huart, msg.scentenceLength);
    }

    TEST_ASSERT(nmea->overflowed == true);
    TEST_ASSERT(NMEA0183__itemsInBuffer(nmea) == (MAX_DATA_BUFFER_SIZE - 1));

    NMEA0183__destroy(nmea);
}

static void test_irq_ignores_short_receive(void) {
    UART_HandleTypeDef huart = {0};
    USART_TypeDef uart_instance = {0};
    DMA_HandleTypeDef dma = {0};
    setup_uart(&huart, &uart_instance, &dma);

    NMEA0183 *nmea = NMEA0183__create(&huart);
    TEST_ASSERT(nmea != NULL);

    NMEA0183Raw msg = {0};
    build_sentence(&msg, '$', "IIMWV,045.0,R");
    copy_sentence_to_dma(nmea, &msg);
    trigger_irq(&huart, 2);

    TEST_ASSERT(NMEA0183__itemsInBuffer(nmea) == 0);

    NMEA0183__destroy(nmea);
}

void Error_Handler(void) {
    g_failures++;
}

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
