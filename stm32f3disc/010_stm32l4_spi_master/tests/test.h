#ifndef __TEST_H
#define __TEST_H

#include <stdint.h>
#include <stdio.h>

extern int test_checks;
extern int test_failures;

#define CHECK(cond)                                                    \
  do                                                                   \
  {                                                                    \
    test_checks++;                                                     \
    if (!(cond))                                                       \
    {                                                                  \
      test_failures++;                                                 \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
    }                                                                  \
  } while (0)

void test_frame_check(void);

#endif /* __TEST_H */
