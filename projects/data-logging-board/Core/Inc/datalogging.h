/*
 * LOGGING.h
 *
 *  Created on: Mar 22, 2025
 *      Author: david
 */

#ifndef INC_DATALOGGING_H_
#define INC_DATALOGGING_H_

#include "stm32u5xx_hal.h"
#include <stdbool.h>

void datalogging_init(ADC_HandleTypeDef *adc_handle);

float time_elapsed(void);

void set_interval(uint32_t interval_ms);
bool interval_elapsed(void);
uint32_t get_millis(void);
float get_seconds(void);

float read_voltage(void);
float read_current(void);
float read_temperature(void);

float get_avg_voltage(void);
float get_avg_current(void);
float get_avg_temperature(void);

#endif /* INC_DATALOGGING_H_ */
