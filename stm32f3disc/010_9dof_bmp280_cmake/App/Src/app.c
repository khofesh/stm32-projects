#include "app.h"

#include <stddef.h>
#include <string.h>

#ifdef NDEBUG
#define DEBUG_LOG 0
#else
#define DEBUG_LOG 1
#include <stdio.h>
#endif

#include "bmp280.h"
#include "bsp_i2c.h"
#include "bsp_spi.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "build_info.h"
#include "i3g4250d.h"
#include "lsm303agr.h"
#include "main.h"
#include "sched.h"
#include "telemetry.h"

/* M1 runs the raw sensors at 25 Hz telemetry; M4 raises the fusion rates. */
#define IMU_PERIOD_MS 10u
#define BARO_PERIOD_MS 40u
#define TELEMETRY_PERIOD_MS 40u
#define DEBUG_PERIOD_MS 1000u
#define HEARTBEAT_PERIOD_MS 500u

#define BUILD_DESC_MAX 32u

static const char s_banner[] = "\r\n010_9dof_bmp280_cmake " BUILD_GIT_DESC "\r\n";
static const char s_build_desc[] = BUILD_GIT_DESC;

_Static_assert(sizeof(BUILD_GIT_DESC) <= BUILD_DESC_MAX, "BUILD_GIT_DESC does not fit the debug frame");

typedef struct
{
  vec3_t accel;
  vec3_t gyro;
  vec3_t mag;
  float pressure_pa;
  float temperature_c;
  uint32_t imu_t_ms;
} sensor_state_t;

static sensor_state_t s_state;
static frame_debug_t s_debug_frame;
static uint16_t s_status;
static uint16_t s_data_seq;
static uint16_t s_debug_seq;
static uint32_t s_max_task_us;
static uint8_t s_overrun;
static uint8_t s_debug_pending;

#if DEBUG_LOG
static void sensors_log(void)
{
  printf("id: i3g4250d=0x%02X lsm303agr=0x%02X/0x%02X bmp280=0x%02X\r\n",
         (unsigned)i3g4250d_id(), (unsigned)lsm303agr_accel_id(),
         (unsigned)lsm303agr_mag_id(), (unsigned)bmp280_chip_id());
}
#else
static void sensors_log(void)
{
}
#endif

static void sensors_init(void)
{
  (void)i3g4250d_init();
  (void)lsm303agr_init();
  (void)bmp280_init();
  sensors_log();
}

void app_init(void)
{
  bsp_time_init();

  if (bsp_uart_init() != 0)
  {
    Error_Handler();
  }
  if (sched_start() != 0)
  {
    Error_Handler();
  }

  sensors_init();

  if (bsp_uart_send_dma((const uint8_t *)s_banner,
                        (uint16_t)(sizeof(s_banner) - 1u)) != 0)
  {
    Error_Handler();
  }
}

static void imu_task(void)
{
  vec3_t v;
  uint32_t t0 = bsp_time_us();

  if (i3g4250d_read(&v) == 0)
  {
    s_state.gyro = v;
    s_status |= STATUS_GYRO_OK;
  }
  else
  {
    s_status &= (uint16_t)~STATUS_GYRO_OK;
  }

  if (lsm303agr_read_accel(&v) == 0)
  {
    s_state.accel = v;
    s_status |= STATUS_ACCEL_OK;
  }
  else
  {
    s_status &= (uint16_t)~STATUS_ACCEL_OK;
  }

  if (lsm303agr_read_mag(&v) == 0)
  {
    s_state.mag = v;
    s_status |= STATUS_MAG_OK;
  }
  else
  {
    s_status &= (uint16_t)~STATUS_MAG_OK;
  }

  s_state.imu_t_ms = bsp_time_ms();

  uint32_t dt = bsp_time_us() - t0;
  if (dt > s_max_task_us)
  {
    s_max_task_us = dt;
  }
}

static void baro_task(void)
{
  float pa = 0.0f;
  float deg_c = 0.0f;
  uint32_t t0 = bsp_time_us();

  if (bmp280_read(&pa, &deg_c) == 0)
  {
    s_state.pressure_pa = pa;
    s_state.temperature_c = deg_c;
    s_status |= STATUS_BARO_OK;
  }
  else
  {
    s_status &= (uint16_t)~STATUS_BARO_OK;
  }

  uint32_t dt = bsp_time_us() - t0;
  if (dt > s_max_task_us)
  {
    s_max_task_us = dt;
  }
}

static void telemetry_task(void)
{
  frame_data_t f;
  uint32_t t0 = bsp_time_us();

  memset(&f, 0, sizeof(f));
  f.seq = s_data_seq++;
  f.t_ms = s_state.imu_t_ms;
  f.q[0] = 1.0f; /* attitude arrives in M4 */
  f.accel = s_state.accel;
  f.gyro = s_state.gyro;
  f.mag = s_state.mag;
  f.pressure_pa = s_state.pressure_pa;
  f.altitude_m = 0.0f;
  f.temperature_cdeg = (int16_t)(s_state.temperature_c * 100.0f);
  f.status = s_status;
  if ((s_status & STATUS_MAG_OK) == 0u)
  {
    f.status |= STATUS_IMU_ONLY;
  }
  if (s_overrun != 0u)
  {
    f.status |= STATUS_OVERRUN;
  }
#if DEBUG_LOG
  f.status |= STATUS_DEBUG_BUILD;
#endif
  f.cpu_load = 0u;
  f.calib_state = 0u;
  f.overruns = (uint16_t)sched_overruns();

  (void)telemetry_send_data(&f);
  s_overrun = 0u;

  uint32_t dt = bsp_time_us() - t0;
  if (dt > s_max_task_us)
  {
    s_max_task_us = dt;
  }
}

static void debug_task(void)
{
  frame_debug_t *d = &s_debug_frame;

  memset(d, 0, sizeof(*d));
  d->seq = s_debug_seq++;
  d->t_ms = bsp_time_ms();
  memcpy(d->build_git_desc, s_build_desc, sizeof(s_build_desc) - 1u);
  d->i2c1_errors = (uint16_t)bsp_i2c_errors(BSP_I2C_BUS_1);
  d->i2c2_errors = (uint16_t)bsp_i2c_errors(BSP_I2C_BUS_2);
  d->spi1_errors = (uint16_t)bsp_spi_errors();
  d->max_loop_us = (uint16_t)s_max_task_us;
  d->cpu_load_permille = 0u; /* DWT in M4 */
  d->free_stack = 0u; /* stack high-water in M5 */

  s_max_task_us = 0u;
  s_debug_pending = 1u;
}

/* The data frame owns the link at the 1 s boundary; send the debug frame as
 * soon as the DMA is idle instead of dropping it. */
static void debug_flush(void)
{
  if ((s_debug_pending == 0u) || (bsp_uart_tx_ready() == 0u))
  {
    return;
  }
  if (telemetry_send_debug(&s_debug_frame) == 0)
  {
    s_debug_pending = 0u;
  }
}

void app_run(void)
{
  uint32_t now = sched_ticks();
  uint32_t next_imu = now;
  uint32_t next_baro = now;
  uint32_t next_telemetry = now;
  uint32_t next_debug = now;
  uint32_t next_heartbeat = now;

  for (;;)
  {
    uint8_t due;

    now = sched_ticks();

    due = sched_due(&next_imu, now, IMU_PERIOD_MS);
    if (due != 0u)
    {
      imu_task();
    }
    if (due == 2u)
    {
      s_overrun = 1u;
    }

    due = sched_due(&next_baro, now, BARO_PERIOD_MS);
    if (due != 0u)
    {
      baro_task();
    }
    if (due == 2u)
    {
      s_overrun = 1u;
    }

    due = sched_due(&next_telemetry, now, TELEMETRY_PERIOD_MS);
    if (due != 0u)
    {
      telemetry_task();
    }
    if (due == 2u)
    {
      s_overrun = 1u;
    }

    due = sched_due(&next_debug, now, DEBUG_PERIOD_MS);
    if (due != 0u)
    {
      debug_task();
    }

    due = sched_due(&next_heartbeat, now, HEARTBEAT_PERIOD_MS);
    if (due != 0u)
    {
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }

    debug_flush();
  }
}
