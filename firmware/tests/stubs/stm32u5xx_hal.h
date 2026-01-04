#ifndef STM32U5XX_HAL_H
#define STM32U5XX_HAL_H

#include <stdint.h>

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
} USART_TypeDef;

typedef struct {
    uint16_t counter;
} DMA_HandleTypeDef;

typedef struct {
    USART_TypeDef *Instance;
    DMA_HandleTypeDef *hdmarx;
} UART_HandleTypeDef;

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

#define USART_CR1_UE 0x00000001U
#define USART_CR2_ADD 0x000000FFU
#define UART_CR2_ADDRESS_LSB_POS 0U
#define UART_IT_CM 0x00000002U
#define USART_ISR_CMF 0x00000001U
#define USART_ICR_CMCF 0x00000001U

#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define READ_BIT(REG, BIT) ((REG) & (BIT))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) \
    do {                                    \
        (REG) = ((REG) & ~(CLEARMASK)) | (SETMASK); \
    } while (0)
#define WRITE_REG(REG, VAL) ((REG) = (VAL))

#define __HAL_UART_ENABLE_IT(huart, it) do { (void)(huart); (void)(it); } while (0)
#define __HAL_UART_CLEAR_OREFLAG(huart) do { (void)(huart); } while (0)
#define __HAL_UART_CLEAR_FEFLAG(huart) do { (void)(huart); } while (0)
#define __HAL_UART_CLEAR_NEFLAG(huart) do { (void)(huart); } while (0)
#define __HAL_UART_CLEAR_PEFLAG(huart) do { (void)(huart); } while (0)
#define __HAL_UART_CLEAR_IDLEFLAG(huart) do { (void)(huart); } while (0)

#define __HAL_DMA_GET_COUNTER(hdma) ((hdma) ? (hdma)->counter : 0U)

static inline HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart,
                                                     uint8_t *pData,
                                                     uint16_t size) {
    (void)huart;
    (void)pData;
    (void)size;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart) {
    (void)huart;
    return HAL_OK;
}

#endif
