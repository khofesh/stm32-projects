#include "app.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef NDEBUG
#define DEBUG_LOG 0
#else
#define DEBUG_LOG 1
#include <stdio.h>
#endif

#include "altitude.h"
#include "baro_filter.h"
#include "bmp280.h"
#include "board_axes.h"
#include "bsp_i2c.h"
#include "calibration.h"
#include "cmd.h"
#include "bsp_spi.h"
#include "bsp_stack.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "build_info.h"
#include "i3g4250d.h"
#include "lsm303agr.h"
#include "madgwick.h"
#include "main.h"
#include "sched.h"
#include "telemetry.h"

/* Full-rate fusion, SPEC §21.3. */
#define IMU_PERIOD_MS 10u
#define BARO_PERIOD_MS 40u
#define TELEMETRY_PERIOD_MS 20u
#define DEBUG_PERIOD_MS 1000u
#define HEARTBEAT_PERIOD_MS 500u
#define OVERRUN_HOLD_MS 1000u

#define IMU_RATE_HZ (1000.0f / (float)IMU_PERIOD_MS)
#define BARO_DT_S ((float)BARO_PERIOD_MS / 1000.0f)

#define FUSION_BETA 0.1f
/* High gain until the attitude converges from identity: ~115 deg/s. */
#define FUSION_BETA_INIT 2.0f
#define FUSION_INIT_UPDATES 200u
/* Longest step integrated after a gap (flash save, overrun). */
#define FUSION_DT_MAX_S 0.5f
#define BARO_FILTER_ALPHA 0.2f
#define BARO_FILTER_BETA 0.01f

#define SENSOR_FLAGS (STATUS_GYRO_OK | STATUS_ACCEL_OK | STATUS_MAG_OK | STATUS_BARO_OK)

#define BUILD_DESC_MAX 32u

/* T-11 latency probe: PC0 on P2 pin 6, configured in CubeMX as LAT_PROBE.
 * Compiled out until the pin exists in main.h. */
#if defined(LAT_PROBE_Pin)
#define PROBE_HIGH() HAL_GPIO_WritePin(LAT_PROBE_GPIO_Port, LAT_PROBE_Pin, GPIO_PIN_SET)
#define PROBE_LOW() HAL_GPIO_WritePin(LAT_PROBE_GPIO_Port, LAT_PROBE_Pin, GPIO_PIN_RESET)
#else
#define PROBE_HIGH() ((void)0)
#define PROBE_LOW() ((void)0)
#endif

/* NFR-15: a failed sensor is re-initialised at most once per second. */
#define REINIT_PERIOD_MS 1000u

typedef struct
{
  uint16_t flag;
  int32_t (*init)(void);
  uint32_t last_ms;
} reinit_t;

static reinit_t s_reinit[] = {
    {STATUS_GYRO_OK, i3g4250d_init, 0u},
    {STATUS_ACCEL_OK, lsm303agr_accel_init, 0u},
    {STATUS_MAG_OK, lsm303agr_mag_init, 0u},
    {STATUS_BARO_OK, bmp280_init, 0u},
};

static const char s_banner[] = "\r\n010_9dof_bmp280_cmake " BUILD_GIT_DESC "\r\n";
static const char s_build_desc[] = BUILD_GIT_DESC;

_Static_assert(sizeof(BUILD_GIT_DESC) <= BUILD_DESC_MAX, "BUILD_GIT_DESC does not fit the debug frame");

typedef struct
{
  vec3_t accel;
  vec3_t gyro;
  vec3_t mag;
  float pressure_pa;
  float altitude_m;
  float temperature_c;
  float q[4];
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
static uint32_t s_busy_cycles;
static uint32_t s_load_window_cycles;
static uint16_t s_cpu_load_permille;
static float s_p0_pa;
static uint8_t s_p0_valid;
static uint32_t s_imu_cycles;
static uint8_t s_imu_primed;
static uint16_t s_fusion_updates;
static uint8_t s_imu_only;

static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

#if DEBUG_LOG
static void sensors_log(void)
{
  printf("id: i3g4250d=0x%02X lsm303agr=0x%02X/0x%02X bmp280=0x%02X\r\n", (unsigned)i3g4250d_id(),
         (unsigned)lsm303agr_accel_id(), (unsigned)lsm303agr_mag_id(), (unsigned)bmp280_chip_id());
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

/* NFR-15: retry a sensor that failed, once per second. A successful init
 * restores its OK flag; the next read proves the bus works. */
static void fault_task(uint32_t now)
{
  for (uint8_t i = 0u; i < (uint8_t)(sizeof(s_reinit) / sizeof(s_reinit[0])); i++)
  {
    reinit_t *r = &s_reinit[i];

    if ((s_status & r->flag) != 0u)
    {
      continue;
    }
    if ((now - r->last_ms) < REINIT_PERIOD_MS)
    {
      continue;
    }
    r->last_ms = now;
    if (r->init() == 0)
    {
      s_init_ok |= r->flag;
    }
  }
}

void app_init(void)
{
  bsp_time_init();
  bsp_stack_init();
  dwt_init();
  s_load_window_cycles = DWT->CYCCNT;

  if (bsp_uart_init() != 0)
  {
    Error_Handler();
  }
  if (sched_start() != 0)
  {
    Error_Handler();
  }

  sensors_init();
  for (uint8_t i = 0u; i < (uint8_t)(sizeof(s_reinit) / sizeof(s_reinit[0])); i++)
  {
    s_reinit[i].last_ms = bsp_time_ms();
  }
  calibration_init(s_init_ok);

  s_state.q[0] = 1.0f;
  s_state.q[1] = 0.0f;
  s_state.q[2] = 0.0f;
  s_state.q[3] = 0.0f;
  s_state.altitude_m = 0.0f;
  madgwick_init(FUSION_BETA_INIT, IMU_RATE_HZ);
  baro_filter_init(BARO_FILTER_ALPHA, BARO_FILTER_BETA, BARO_DT_S);
  s_p0_pa = calibration_p0_pa();
  s_p0_valid = (s_p0_pa > 0.0f) ? 1u : 0u;

  if (bsp_uart_send_dma((const uint8_t *)s_banner, (uint16_t)(sizeof(s_banner) - 1u)) != 0)
  {
    Error_Handler();
  }
}

/* Step length from CYCCNT: SysTick misses ticks while a flash erase stalls
 * the core, the cycle counter does not. */
static float imu_dt_s(void)
{
  uint32_t c = DWT->CYCCNT;
  float dt = (float)(c - s_imu_cycles) / (float)SystemCoreClock;

  s_imu_cycles = c;
  if (s_imu_primed == 0u)
  {
    s_imu_primed = 1u;
    return 0.0f;
  }
  return (dt > FUSION_DT_MAX_S) ? FUSION_DT_MAX_S : dt;
}

static void fusion_update(void)
{
  uint8_t acc_ok = ((s_status & STATUS_ACCEL_OK) != 0u) ? 1u : 0u;
  uint8_t mag_ok = (((s_status & STATUS_MAG_OK) != 0u) && ((calibration_status() & STATUS_MAG_CAL) != 0u)) ? 1u : 0u;
  float dt = imu_dt_s();

  s_imu_only = (mag_ok == 0u) ? 1u : 0u;

  /* A stale rate would keep integrating: skip the step. The next good
   * sample restarts at the nominal dt. */
  if ((s_status & STATUS_GYRO_OK) == 0u)
  {
    s_imu_primed = 0u;
    return;
  }

  madgwick_update_dt(&s_state.gyro, (acc_ok != 0u) ? &s_state.accel : NULL,
                     ((acc_ok != 0u) && (mag_ok != 0u)) ? &s_state.mag : NULL, dt);
  madgwick_get_quat(s_state.q);

  if (s_fusion_updates < FUSION_INIT_UPDATES)
  {
    s_fusion_updates++;
    if (s_fusion_updates == FUSION_INIT_UPDATES)
    {
      madgwick_set_beta(FUSION_BETA);
    }
  }
}

static void imu_task(void)
{
  vec3_t v;
  int32_t st;

  PROBE_HIGH();
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

  PROBE_LOW();
  fusion_update();

  s_state.imu_t_ms = bsp_time_ms();
}

static void baro_task(void)
{
  float pa = 0.0f;
  float deg_c = 0.0f;
  float p0 = calibration_p0_pa();
  int32_t st;

  if (p0 <= 0.0f)
  {
    p0 = ALTITUDE_SEA_LEVEL_PA;
  }
  if ((s_p0_valid == 0u) || (fabsf(p0 - s_p0_pa) > 0.5f))
  {
    /* A new 'B' reference moves the zero: restart the filter instead of
     * letting it drift through the step. */
    baro_filter_init(BARO_FILTER_ALPHA, BARO_FILTER_BETA, BARO_DT_S);
    s_p0_pa = p0;
    s_p0_valid = 1u;
  }

  st = ((s_init_ok & STATUS_BARO_OK) != 0u) ? bmp280_read(&pa, &deg_c) : -ENODEV;
  if (st == 0)
  {
    s_state.pressure_pa = pa;
    s_state.temperature_c = deg_c;
    calibration_feed_baro(pa);
    s_state.altitude_m = baro_filter_update(altitude_from_pressure(pa, p0));
  }
  status_set(STATUS_BARO_OK, st);
}

static void telemetry_task(void)
{
  frame_data_t f;

  memset(&f, 0, sizeof(f));
  f.seq = s_data_seq++;
  f.t_ms = s_state.imu_t_ms;
  f.q[0] = s_state.q[0];
  f.q[1] = s_state.q[1];
  f.q[2] = s_state.q[2];
  f.q[3] = s_state.q[3];
  f.accel = s_state.accel;
  f.gyro = s_state.gyro;
  f.mag = s_state.mag;
  f.pressure_pa = s_state.pressure_pa;
  f.altitude_m = s_state.altitude_m;
  f.temperature_cdeg = (int16_t)(s_state.temperature_c * 100.0f);
  f.status = s_status & SENSOR_FLAGS;
  f.status |= calibration_status();
  if (calibration_active() != 0u)
  {
    f.status |= STATUS_CALIBRATING;
  }
  if (s_imu_only != 0u)
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
  f.cpu_load = (uint8_t)(s_cpu_load_permille / 10u);
  f.calib_state = calibration_state();
  f.overruns = (uint16_t)s_overruns;

  /* A dropped frame is counted in telemetry_skipped() and reported in the
   * next one. */
  (void)telemetry_send_data(&f);
}

static void debug_task(void)
{
  frame_debug_t *d = &s_debug_frame;
  uint32_t now_cycles = DWT->CYCCNT;
  uint32_t elapsed_cycles = now_cycles - s_load_window_cycles;

  if (elapsed_cycles > 0u)
  {
    uint64_t load = ((uint64_t)s_busy_cycles * 1000u) / (uint64_t)elapsed_cycles;

    s_cpu_load_permille = (load > 1000u) ? 1000u : (uint16_t)load;
  }
  s_busy_cycles = 0u;
  s_load_window_cycles = now_cycles;

  memset(d, 0, sizeof(*d));
  d->seq = s_debug_seq++;
  d->t_ms = bsp_time_ms();
  memcpy(d->build_git_desc, s_build_desc, sizeof(s_build_desc) - 1u);
  d->i2c1_errors = (uint16_t)bsp_i2c_errors(BSP_I2C_BUS_1);
  d->i2c2_errors = (uint16_t)bsp_i2c_errors(BSP_I2C_BUS_2);
  d->spi1_errors = (uint16_t)bsp_spi_errors();
  d->max_loop_us = (s_max_loop_us > UINT16_MAX) ? UINT16_MAX : (uint16_t)s_max_loop_us;
  d->cpu_load_permille = s_cpu_load_permille;
  d->free_stack = (uint16_t)bsp_stack_free_bytes();

  s_max_loop_us = 0u;
  s_debug_pending = 1u;
}

/* The data frame owns the link at the 1 s boundary; send the debug frame as
 * soon as the DMA is idle instead of dropping it. */
static uint8_t debug_flush(void)
{
  if ((s_debug_pending == 0u) || (bsp_uart_tx_ready() == 0u))
  {
    return 0u;
  }
  if (telemetry_send_debug(&s_debug_frame) == 0)
  {
    s_debug_pending = 0u;
  }
  return 1u;
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
    uint32_t c0 = DWT->CYCCNT;
    uint32_t stall_cycles = 0u;
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

    fault_task(now);

    /* Idle work: one-byte calibration commands from the host. */
    if (cmd_poll() != 0u)
    {
      ran = 1u;
    }

    if (calibration_take_save_request() != 0u)
    {
      uint32_t since = sched_ticks();
      uint32_t ms = 0u;
      uint32_t s0 = DWT->CYCCNT;

      /* Failure is reported through calib_state. */
      (void)calibration_save(&ms);
      sched_compensate(since, ms);

      /* The flash stall is a commanded gap, not an overrun: realign. */
      now = sched_ticks();
      next_imu = now;
      next_baro = now;
      next_telemetry = now;
      stall_cycles = DWT->CYCCNT - s0;
    }

    if (debug_flush() != 0u)
    {
      ran = 1u;
    }

    /* The flash erase stall is not CPU load. Empty polling spins are idle. */
    if (ran != 0u)
    {
      uint32_t busy = (DWT->CYCCNT - c0) - stall_cycles;
      uint32_t us = (uint32_t)(((uint64_t)busy * 1000000u) / SystemCoreClock);

      s_busy_cycles += busy;
      if (us > s_max_loop_us)
      {
        s_max_loop_us = us;
      }
    }
  }
}
