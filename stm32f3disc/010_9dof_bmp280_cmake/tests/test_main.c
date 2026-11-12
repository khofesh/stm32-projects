#include "test.h"

int test_checks;
int test_failures;

int main(void)
{
  test_crc16();
  test_axis_map();

  printf("%d check(s), %d failure(s)\n", test_checks, test_failures);
  return (test_failures == 0) ? 0 : 1;
}
