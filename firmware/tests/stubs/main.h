#ifndef MAIN_H
#define MAIN_H

#include "stm32u5xx_hal.h"

#define GPIO_PIN_2  (1U << 2)
#define GPIO_PIN_7  (1U << 7)

#define GPIOG ((GPIO_TypeDef *)0)
#define GPIOC ((GPIO_TypeDef *)0)

extern uint8_t g_fw_enable_debug_prints;
extern uint8_t g_fw_enable_can_prints;

void Error_Handler(void);

#endif
