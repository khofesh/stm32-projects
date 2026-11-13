#include "app.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef NDEBUG
#define DEBUG_LOG 0
#else
#define DEBUG_LOG 1
#include <stdio.h>
#endif

#include "bmp280.h"
#include "board_axes.h"
#include "bsp_i2c.h"
#include "calibration.h"
#include "cmd.h"
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
#define OVERRUN_HOLD_MS 1000u

#define SENSOR_FLAGS (STATUS_GYRO_OK | STATUS_ACCEL_OK | STATUS_MAG_OK | STATUS_BARO_OK)

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
static uint16_t s_init_ok; /* sensors that passed ID check and configuration */
static uint16_t s_data_seq;
static uint16_t s_debug_seq;
static uint32_t s_max_loop_us;
static uint32_t s_overruns;
static uint32_t s_overrun_ms;
static uint8_t s_overrun_seen;
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
  s_init_ok = 0u;
  if (i3g4250d_init() == 0)
  {
    s_init_ok |= STATUS_GYRO_OK;
  }
  if (lsm303agr_accel_init() == 0)
  {
    s_init_ok |= STATUS_ACCEL_OK;
  }
  if (lsm303agr_mag_init() == 0)
  {
    s_init_ok |= STATUS_MAG_OK;
  }
  if (bmp280_init() == 0)
  {
    s_init_ok |= STATUS_BARO_OK;
  }
  sensors_log();
}

/* A sensor is OK only if it initialised and its last read succeeded. */
static void status_set(uint16_t flag, int32_t read_status)
{
  if ((read_status == 0) && ((s_init_ok & flag) != 0u))
  {
    s_status |= flag;
  }
  else
  {
    s_status &= (uint16_t)~flag;
  }
}

static void overrun_note(void)
{
  s_overruns++;
  s_overrun_ms = bsp_time_ms();
  s_overrun_seen = 1u;
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
  calibration_init(s_init_ok);

  if (bsp_uart_send_dma((const uint8_t *)s_banner,
                        (uint16_t)(sizeof(s_banner) - 1u)) != 0)
  {
    Error_Handler();
  }
}

static void imu_task(void)
{
  vec3_t v;
  int32_t st;

  st = ((s_init_ok & STATUS_GYRO_OK) != 0u) ? i3g4250d_read(&v) : -ENODEV;
  if (st == 0)
  {
    board_axes_apply_gyro(&v);
    calibration_feed_gyro(&v);
    calibration_apply_gyro(&v);
    s_state.gyro = v;
  }
  status_set(STATUS_GYRO_OK, st);

  st = ((s_init_ok & STATUS_ACCEL_OK) != 0u) ? lsm303agr_read_accel(&v) : -ENODEV;
  if (st == 0)
  {
    board_axes_apply_accel(&v);
    calibration_feed_accel(&v);
    calibration_apply_accel(&v);
    s_state.accel = v;
  }
  status_set(STATUS_ACCEL_OK, st);

  st = ((s_init_ok & STATUS_MAG_OK) != 0u) ? lsm303agr_read_mag(&v) : -ENODEV;
  if (st == 0)
  {
    board_axes_apply_mag(&v);
    calibration_feed_mag(&v);
    calibration_apply_mag(&v);
    s_state.mag = v;
  }
  status_set(STATUS_MAG_OK, st);

  s_state.imu_t_ms = bsp_time_ms();
}

static void baro_task(void)
{
  float pa = 0.0f;
  float deg_c = 0.0f;
  int32_t st;

  st = ((s_init_ok & STATUS_BARO_OK) != 0u) ? bmp280_read(&pa, &deg_c) : -ENODEV;
  if (st == 0)
  {
    s_state.pressure_pa = pa;
    s_state.temperature_c = deg_c;
    calibration_feed_baro(pa);
  }
  status_set(STATUS_BARO_OK, st);
}

static void telemetry_task(void)
{
  frame_data_t f;

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
  f.status = s_status & SENSOR_FLAGS;
  f.status |= calibration_status();
  if (calibration_active() != 0u)
  {
    f.status |= STATUS_CALIBRATING;
  }
  if ((s_status & STATUS_MAG_OK) == 0u)
  {
    f.status |= STATUS_IMU_ONLY;
  }
  if ((s_overrun_seen != 0u) && ((bsp_time_ms() - s_overrun_ms) < OVERRUN_HOLD_MS))
  {
    f.status |= STATUS_OVERRUN;
  }
#if DEBUG_LOG
  f.status |= STATUS_DEBUG_BUILD;
#endif
  f.cpu_load = 0u;
  f.calib_state = calibration_state();
  f.overruns = (uint16_t)s_overruns;

  /* A dropped frame is counted in telemetry_skipped() and reported in the
   * next one. */
  (void)telemetry_send_data(&f);
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
  d->max_loop_us = (s_max_loop_us > UINT16_MAX) ? UINT16_MAX : (uint16_t)s_max_loop_us;
  d->cpu_load_permille = 0u; /* DWT in M4 */
  d->free_stack = 0u; /* stack high-water in M5 */

  s_max_loop_us = 0u;
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
    uint32_t t0 = bsp_time_us();
    uint8_t ran = 0u;
    uint8_t due;

    now = sched_ticks();

    due = sched_due(&next_imu, now, IMU_PERIOD_MS);
    if (due != 0u)
    {
      imu_task();
      ran = 1u;
    }
    if (due == 2u)
    {
      overrun_note();
    }

    due = sched_due(&next_baro, now, BARO_PERIOD_MS);
    if (due != 0u)
    {
      baro_task();
      ran = 1u;
    }
    if (due == 2u)
    {
      overrun_note();
    }

    due = sched_due(&next_telemetry, now, TELEMETRY_PERIOD_MS);
    if (due != 0u)
    {
      telemetry_task();
      ran = 1u;
    }
    if (due == 2u)
    {
      overrun_note();
    }

    /* Bookkeeping tasks: lateness is not an overrun. */
    if (sched_due(&next_debug, now, DEBUG_PERIOD_MS) != 0u)
    {
      debug_task();
      ran = 1u;
    }
    if (sched_due(&next_heartbeat, now, HEARTBEAT_PERIOD_MS) != 0u)
    {
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }

    /* Idle work: one-byte calibration commands from the host. */
    cmd_poll();

    if (calibration_take_save_request() != 0u)
    {
      uint32_t since = sched_ticks();
      uint32_t ms = 0u;

      /* Failure is reported through calib_state. */
      (void)calibration_save(&ms);
      sched_compensate(since, ms);

      /* The flash stall is a commanded gap, not an overrun: realign. */
      now = sched_ticks();
      next_imu = now;
      next_baro = now;
      next_telemetry = now;
    }

    debug_flush();

    if (ran != 0u)
    {
      uint32_t dt = bsp_time_us() - t0;

      if (dt > s_max_loop_us)
      {
        s_max_loop_us = dt;
      }
    }
  }
}
