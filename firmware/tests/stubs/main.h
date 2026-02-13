#ifndef MAIN_H
#define MAIN_H

#include "stm32u5xx_hal.h"

#define GPIO_PIN_2  (1U << 2)
#define GPIO_PIN_7  (1U << 7)

#define GPIOG ((GPIO_TypeDef *)0)
#define GPIOC ((GPIO_TypeDef *)0)

void Error_Handler(void);

#endif
