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

#define CHECK_EQ_U16(actual, expected)                                       \
  do                                                                         \
  {                                                                          \
    uint16_t got_ = (uint16_t)(actual);                                      \
    uint16_t want_ = (uint16_t)(expected);                                   \
    test_checks++;                                                           \
    if (got_ != want_)                                                       \
    {                                                                        \
      test_failures++;                                                       \
      printf("FAIL %s:%d: %s = 0x%04X, expected 0x%04X\n",                   \
             __FILE__, __LINE__, #actual, (unsigned)got_, (unsigned)want_);  \
    }                                                                        \
  } while (0)

#define CHECK_NEAR_F(actual, expected, tol)                                  \
  do                                                                         \
  {                                                                          \
    float got_ = (actual);                                                   \
    float want_ = (expected);                                                \
    float d_ = got_ - want_;                                                 \
    test_checks++;                                                           \
    if ((d_ > (tol)) || (d_ < -(tol)))                                       \
    {                                                                        \
      test_failures++;                                                       \
      printf("FAIL %s:%d: %s = %g, expected %g\n",                           \
             __FILE__, __LINE__, #actual, (double)got_, (double)want_);      \
    }                                                                        \
  } while (0)

void test_crc16(void);
void test_axis_map(void);
void test_bmp280(void);
void test_frame(void);

#endif /* __TEST_H */
