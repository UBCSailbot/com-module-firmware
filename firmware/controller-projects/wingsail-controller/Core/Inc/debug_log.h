#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdio.h>

/*
 * Set to 1 to enable runtime debug logging across modules.
 * Set to 0 to compile out debug prints.
 */
#define APP_DEBUG_LOG 0

#if APP_DEBUG_LOG
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...) ((void)0)
#endif

#endif /* DEBUG_LOG_H */
