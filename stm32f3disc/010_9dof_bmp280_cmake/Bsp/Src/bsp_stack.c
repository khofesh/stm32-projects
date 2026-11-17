#include "bsp_stack.h"

#include <stddef.h>

#include "main.h"

#define STACK_PATTERN 0xA5A5A5A5u
/* Keep the frame of this call out of the painted region. */
#define STACK_GUARD_BYTES 64u

/* Bottom of the 1 KB stack reservation (linker script). */
extern uint32_t _sstack[];

static uint32_t *s_lo;
static uint32_t *s_hi;

void bsp_stack_init(void)
{
  uint32_t sp = __get_MSP();
  uintptr_t lo = (uintptr_t)_sstack;
  uintptr_t hi = (uintptr_t)(sp - STACK_GUARD_BYTES) & ~(uintptr_t)3u;

  s_lo = NULL;
  s_hi = NULL;
  if (hi <= lo)
  {
    return;
  }
  for (uintptr_t p = lo; p < hi; p += sizeof(uint32_t))
  {
    *(uint32_t *)p = STACK_PATTERN;
  }
  s_lo = (uint32_t *)lo;
  s_hi = (uint32_t *)hi;
}

uint32_t bsp_stack_free_bytes(void)
{
  const uint32_t *p;

  if (s_lo == NULL)
  {
    return 0u;
  }
  for (p = s_lo; p < s_hi; p++)
  {
    if (*p != STACK_PATTERN)
    {
      break;
    }
  }
  return (uint32_t)((uintptr_t)p - (uintptr_t)s_lo);
}
