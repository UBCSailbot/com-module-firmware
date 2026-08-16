#ifndef TEST_ASSERT_H
#define TEST_ASSERT_H

#include <stdio.h>

/**
 * @brief Record a test assertion failure.
 *
 * @param condition Expression result to evaluate.
 * @param expr String form of the expression.
 * @param file Source file where the assertion occurred.
 * @param line Line number where the assertion occurred.
 * @return void
 */
static inline void test_assert(int condition, const char *expr,
                               const char *file, int line, int *failures) {
  if (!condition) {
    printf("FAIL: %s:%d: %s\n", file, line, expr);
    (*failures)++;
  }
}

#define TEST_ASSERT(failures_ptr, cond) \
  test_assert((cond), #cond, __FILE__, __LINE__, (failures_ptr))

#endif
