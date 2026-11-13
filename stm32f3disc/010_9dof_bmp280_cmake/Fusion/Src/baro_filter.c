#include "baro_filter.h"

#include <stdint.h>

static float s_alpha = 0.2f;
static float s_beta = 0.01f;
static float s_dt = 0.04f;
static float s_alt;
static float s_vel;
static uint8_t s_primed;

void baro_filter_init(float alpha, float beta, float dt)
{
  s_alpha = (alpha > 0.0f) ? alpha : 0.0f;
  s_beta = (beta > 0.0f) ? beta : 0.0f;
  s_dt = (dt > 0.0f) ? dt : 0.04f;
  s_alt = 0.0f;
  s_vel = 0.0f;
  s_primed = 0u;
}

float baro_filter_update(float alt_m)
{
  float pred;
  float residual;

  if (s_primed == 0u)
  {
    s_alt = alt_m;
    s_vel = 0.0f;
    s_primed = 1u;
    return s_alt;
  }

  pred = s_alt + (s_vel * s_dt);
  residual = alt_m - pred;
  s_alt = pred + (s_alpha * residual);
  s_vel += (s_beta * residual) / s_dt;
  return s_alt;
}
