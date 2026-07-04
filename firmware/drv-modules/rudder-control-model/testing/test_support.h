#ifndef TEST_SUPPORT_H_
#define TEST_SUPPORT_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../RUDDER.h"

extern uint32_t fake_tick_ms;

void reset_test_controller(void);

#define ASSERT_TRUE(expr)                                                        \
    do {                                                                         \
        if (!(expr)) {                                                           \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr);              \
            return false;                                                        \
        }                                                                        \
    } while (0)

#define ASSERT_EQ_INT(expected, actual)                                          \
    do {                                                                         \
        int expected_value = (int)(expected);                                    \
        int actual_value = (int)(actual);                                        \
        if (expected_value != actual_value) {                                    \
            printf(                                                              \
                "FAIL %s:%d: expected %d, got %d\n",                            \
                __FILE__,                                                        \
                __LINE__,                                                        \
                expected_value,                                                  \
                actual_value                                                     \
            );                                                                   \
            return false;                                                        \
        }                                                                        \
    } while (0)

#define ASSERT_NEAR(expected, actual, tolerance)                                 \
    do {                                                                         \
        float expected_value = (float)(expected);                                \
        float actual_value = (float)(actual);                                    \
        float tolerance_value = (float)(tolerance);                              \
        if (fabsf(expected_value - actual_value) > tolerance_value) {            \
            printf(                                                              \
                "FAIL %s:%d: expected %.3f, got %.3f\n",                        \
                __FILE__,                                                        \
                __LINE__,                                                        \
                expected_value,                                                  \
                actual_value                                                     \
            );                                                                   \
            return false;                                                        \
        }                                                                        \
    } while (0)

typedef bool (*test_fn)(void);

typedef struct {
    const char *name;
    test_fn run;
} TestCase;

static inline int run_tests(const TestCase *tests, unsigned int test_count)
{
    int failures = 0;

    for (unsigned int i = 0; i < test_count; i++) {
        if (tests[i].run()) {
            printf("PASS %s\n", tests[i].name);
        } else {
            printf("FAIL %s\n", tests[i].name);
            failures++;
        }
    }

    if (failures > 0) {
        printf("%d test(s) failed\n", failures);
        return 1;
    }

    printf("All tests passed\n");
    return 0;
}

#endif /* TEST_SUPPORT_H_ */
