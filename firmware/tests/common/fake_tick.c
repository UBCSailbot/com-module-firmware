#include "fake_tick.h"

static uint32_t g_fake_tick_ms = 0U;

void fake_tick_set(uint32_t ms) { g_fake_tick_ms = ms; }

void fake_tick_advance(uint32_t ms) { g_fake_tick_ms += ms; }

uint32_t HAL_GetTick(void) { return g_fake_tick_ms; }
