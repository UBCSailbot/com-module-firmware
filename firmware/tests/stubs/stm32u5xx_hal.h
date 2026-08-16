#ifndef STM32U5XX_HAL_H
#define STM32U5XX_HAL_H

#include <stdint.h>

/* On target this comes from the CMSIS compiler header and marks a symbol as
 * overridable. Host builds link only one definition, so it expands to nothing. */
#ifndef __weak
#define __weak
#endif

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

#define RESET 0U

#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U

typedef struct {
    uint32_t dummy;
} GPIO_TypeDef;

typedef uint32_t GPIO_PinState;

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

/**
 * @brief Stub HAL tick getter.
 *
 * @param void
 * @return Stub tick value in milliseconds.
 */
static inline uint32_t HAL_GetTick(void) {
    return 0U;
}

/**
 * @brief Stub HAL UART DMA receive.
 *
 * @param huart UART handle to configure.
 * @param pData Destination buffer.
 * @param size Number of bytes to receive.
 * @return HAL_OK for host tests.
 */
static inline HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart,
                                                     uint8_t *pData,
                                                     uint16_t size) {
    (void)huart;
    (void)pData;
    (void)size;
    return HAL_OK;
}

/**
 * @brief Stub HAL UART DMA stop.
 *
 * @param huart UART handle to stop.
 * @return HAL_OK for host tests.
 */
static inline HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart) {
    (void)huart;
    return HAL_OK;
}

/**
 * @brief Stub HAL GPIO write.
 *
 * @param GPIOx GPIO port.
 * @param GPIO_Pin GPIO pin mask.
 * @param PinState Pin state to write.
 * @return void
 */
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                                     GPIO_PinState PinState) {
    (void)GPIOx;
    (void)GPIO_Pin;
    (void)PinState;
}

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t CNT;
    volatile uint32_t ARR;
} TIM_TypeDef;

typedef struct {
    TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

/* CANFD uses TIM7 for the 10s heartbeat and compares htim->Instance against it.
 * Mirror CMSIS and make it a memory-mapped address (the real STM32U5 TIM7 base)
 * so no linker symbol is needed; the host never dereferences it. */
#define TIM7 ((TIM_TypeDef *)0x40001400UL)

/**
 * @brief Stub HAL timer base start in interrupt mode.
 *
 * @param htim Timer handle to start.
 * @return HAL_OK for host tests.
 */
static inline HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim) {
    (void)htim;
    return HAL_OK;
}

#endif
