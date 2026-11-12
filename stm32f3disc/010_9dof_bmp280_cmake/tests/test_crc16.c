#include "crc16.h"
#include "test.h"

void test_crc16(void)
{
  static const uint8_t vector[] = "123456789";
  static const uint8_t empty[1] = {0u};

  /* CRC-16/CCITT-FALSE published check value. */
  CHECK_EQ_U16(crc16_ccitt(vector, 9u), 0x29B1u);

  /* Empty input returns the initial value. */
  CHECK_EQ_U16(crc16_ccitt(empty, 0u), 0xFFFFu);

  /* Single zero byte: 0xFFFF ^ 0x0000 shifted eight times. */
  CHECK_EQ_U16(crc16_ccitt(empty, 1u), 0xE1F0u);
}
