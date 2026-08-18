#ifndef FAKE_TICK_H
#define FAKE_TICK_H

#include <stdint.h>

/**
 * @brief Set the value returned by HAL_GetTick.
 *
 * @param ms Tick value in milliseconds.
 * @return void
 */
void fake_tick_set(uint32_t ms);

/**
 * @brief Advance the value returned by HAL_GetTick.
 *
 * @param ms Milliseconds to add.
 * @return void
 */
void fake_tick_advance(uint32_t ms);

#endif
