#include "test.h"

int test_checks;
int test_failures;

int main(void)
{
  test_frame_check();

  printf("%d check(s), %d failure(s)\n", test_checks, test_failures);
  return (test_failures == 0) ? 0 : 1;
}
