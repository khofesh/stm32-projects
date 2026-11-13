#include "test.h"

int test_checks;
int test_failures;

int main(void)
{
  test_crc16();
  test_axis_map();
  test_bmp280();
  test_frame();
  test_calibration();
  test_madgwick();
  test_altitude();
  test_baro_filter();

  printf("%d check(s), %d failure(s)\n", test_checks, test_failures);
  return (test_failures == 0) ? 0 : 1;
}
