/*
 * datalogging.c
 *
 *  Created on: Mar 22, 2025
 *      Author: david
 */

#include "datalogging.h"
#include <math.h>
#include <stdbool.h>

#define NUM_READINGS 30

static ADC_HandleTypeDef *hadc;

static uint32_t startTime = 0;  // ⏱️ Start time in ms

static float voltageBuffer[NUM_READINGS] = {0};
static float currentBuffer[NUM_READINGS] = {0};
static float temperatureBuffer[NUM_READINGS] = {0};
static int bufferIndex = 0;
static bool bufferFilled = false;

static uint32_t interval = 1000;          // default interval
static uint32_t lastTick = 0;             // stores last trigger time
static uint32_t startTick = 0;            // for elapsed time since start



static uint32_t read_adc_channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_5CYCLE;

    HAL_ADC_Stop(hadc);
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) return 0;
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 100);
    uint32_t value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return value;
}

void datalogging_init(ADC_HandleTypeDef *adc_handle) {
    hadc = adc_handle;
    startTime = HAL_GetTick();  // Set base time
    lastTick = startTick;                // initialize interval tracking
}

float time_elapsed(void) {
    uint32_t now = HAL_GetTick();         // milliseconds
    return (now - startTime) / 1000.0;   // seconds
}

void set_interval(uint32_t interval_ms) {
    interval = interval_ms;
    lastTick = HAL_GetTick();            // reset interval tracking
}

bool interval_elapsed(void) {
    uint32_t now = HAL_GetTick();
    if ((now - lastTick) >= interval) {
        lastTick = now;
        return true;
    }
    return false;
}

uint32_t get_millis(void) {
    return HAL_GetTick() - startTick;
}

float get_seconds(void) {
    return (HAL_GetTick() - startTick) / 1000.0f;
}

float read_voltage(void) {
    uint32_t adc_voltage = read_adc_channel(ADC_CHANNEL_1);
    float voltage;
    if (adc_voltage < 10600) {
        voltage = -17.6 + 0.00237 * adc_voltage;
    } else {
        voltage = 249 - 0.0475 * adc_voltage + 0.00000234 * pow(adc_voltage, 2);
    }
    voltageBuffer[bufferIndex] = voltage;
    return voltage;
}

float read_current(void) {
    uint32_t adc_current = read_adc_channel(ADC_CHANNEL_2);
    float current = -551 + 0.167 * adc_current
        - 0.0000171 * pow(adc_current, 2)
        + 0.000000000595 * pow(adc_current, 3);
    currentBuffer[bufferIndex] = current;
    return current;
}

float read_temperature(void) {
    uint32_t adc_voltage = read_adc_channel(ADC_CHANNEL_1);
    uint32_t adc_temp = read_adc_channel(ADC_CHANNEL_15);

    float voltage;
    if (adc_voltage < 10600) {
        voltage = -17.6 + 0.00237 * adc_voltage;
    } else {
        voltage = 249 - 0.0475 * adc_voltage + 0.00000234 * pow(adc_voltage, 2);
    }

    float VRT = voltage * (adc_temp / 16384.0);
    float VR = voltage - VRT;
    if (VR == 0) return -273.15;

    float RT = VRT / (VR / 10000.0);
    float ln = log(RT / 10000.0);
    float TX = 1.0 / ((ln / 3977) + (1.0 / 298.15));
    float tempC = TX - 273.15;

    temperatureBuffer[bufferIndex] = tempC;

    bufferIndex = (bufferIndex + 1) % NUM_READINGS;
    if (bufferIndex == 0) bufferFilled = true;

    return tempC;
}

float get_avg_voltage(void) {
    int count = bufferFilled ? NUM_READINGS : bufferIndex;
    float sum = 0;
    for (int i = 0; i < count; i++) sum += voltageBuffer[i];
    return sum / count;
}

float get_avg_current(void) {
    int count = bufferFilled ? NUM_READINGS : bufferIndex;
    float sum = 0;
    for (int i = 0; i < count; i++) sum += currentBuffer[i];
    return sum / count;
}

float get_avg_temperature(void) {
    int count = bufferFilled ? NUM_READINGS : bufferIndex;
    float sum = 0;
    for (int i = 0; i < count; i++) sum += temperatureBuffer[i];
    return sum / count;
}

