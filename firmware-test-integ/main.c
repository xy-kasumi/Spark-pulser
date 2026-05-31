// SPDX-License-Identifier: BSD-2-Clause
/**
 * Core0: slow, complex (IRQ allowed) — I2C slave, host-facing register
 * interface. Core1: fast, simple (no IRQ) — pulse loop.
 *
 * Register map: docs/i2c-registers.md. Window model: docs/operation.md.
 *
 * Shared state lives in csec_cfg (config + run/WDT control, host<->core1) and
 * csec_stat (cut-mode window counts, core1->core0). `fault` is a sticky atomic
 * raised by either core; once set, the device can never run again.
 */
#include <stdatomic.h>
#include <stdint.h>
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/sync.h"

static const uint8_t MODE_PROBE = 0;  // cut mode is the complementary value 1

// Permanent fault: core0 & core1 shared, sticky. Device cannot run while set,
// and there is no way to clear it (FAULT.fault write is a no-op).
static _Atomic bool fault = false;

// Config + run control: written by core0 (I2C), read by core1 each iteration.
// `run` is bidirectional — host starts/stops it, the device clears it on probe
// completion or WDT timeout.
static critical_section_t csec_cfg;
static bool cfg_run;  // device is running
static bool cfg_wdt;  // WDT timeout latched; recoverable, host clears via FAULT
static bool cfg_probe_detected;      // probe-mode conduction result
static absolute_time_t cfg_wdt_ref;  // last RES0 read / run start, for WDT
static uint8_t cfg_mode;
static uint8_t cfg_curr;           // stored only; clamped to CURR_SUPPORTED
static uint8_t cfg_dur_code;       // 0..15 -> pulse duration (code+1)*50us
static uint8_t cfg_max_duty_code;  // 0..15 -> max duty (code+1)/16

// Cut-mode window counts since last RES0 read: written by core1, read & reset
// by core0 on RES0 read. The WDT caps the reset interval at ~51ms and the
// fastest window is ~70us, so each count stays under ~730 — well within uint16.
static critical_section_t csec_stat;
static uint16_t stat_open = 0;
static uint16_t stat_short = 0;
static uint16_t stat_good = 0;

static const uint8_t CURR_SUPPORTED = 10;  // only current this rig can pulse

static const uint8_t MODE_ON_RESET = 0;     // probe
static const uint8_t CURR_ON_RESET = 0x0a;  // 10
static const uint8_t TIM_ON_RESET = 0x71;   // max_duty=7, dur=1

////////////////////////////////////////////////////////////////////////////////
// Core0: register read/write & I2C slave.

static const uint8_t REG_CTRL = 0x01;
static const uint8_t REG_MODE = 0x02;
static const uint8_t REG_CURR = 0x03;
static const uint8_t REG_TIM = 0x04;
static const uint8_t REG_RES0 = 0x08;
static const uint8_t REG_RES1 = 0x09;
static const uint8_t REG_FAULT = 0x10;

// num_good captured at the last RES0 read, returned by the following RES1 read.
// Lets the host fetch (RES0, RES1) atomically via a sequential read.
static uint8_t res1_latch = 0;

static bool i2c_ptr_written = false;
static uint8_t i2c_reg_ptr = 0;

void write_reg(uint8_t reg, uint8_t val) {
  switch (reg) {
    case REG_CTRL: {
      bool run_req = val & 0x01;
      critical_section_enter_blocking(&csec_cfg);
      if (!run_req) {
        cfg_run = false;
      } else if (!cfg_run && !atomic_load(&fault)) {
        // Rising edge of run: arm WDT and clear last run's results. A latched
        // wdt bit does not block restart — it is recoverable, unlike fault.
        cfg_run = true;
        cfg_wdt_ref = get_absolute_time();
        cfg_probe_detected = false;
        critical_section_enter_blocking(&csec_stat);
        stat_open = stat_short = stat_good = 0;
        critical_section_exit(&csec_stat);
      }
      critical_section_exit(&csec_cfg);
      break;
    }
    case REG_MODE:
      critical_section_enter_blocking(&csec_cfg);
      if (!cfg_run) {  // write fails while running
        cfg_mode = val & 0x01;
      }
      critical_section_exit(&csec_cfg);
      break;
    case REG_CURR:
      critical_section_enter_blocking(&csec_cfg);
      if (!cfg_run) {
        cfg_curr = CURR_SUPPORTED;  // clamp to the only supported value
      }
      critical_section_exit(&csec_cfg);
      break;
    case REG_TIM:
      critical_section_enter_blocking(&csec_cfg);
      if (!cfg_run) {
        cfg_max_duty_code = (val >> 4) & 0x0f;
        cfg_dur_code = val & 0x0f;
      }
      critical_section_exit(&csec_cfg);
      break;
    case REG_FAULT:
      if (val & 0x02) {  // wdt bit: write 1 clears
        critical_section_enter_blocking(&csec_cfg);
        cfg_wdt = false;
        critical_section_exit(&csec_cfg);
      }
      // fault bit write is a no-op.
      break;
  }
}

uint8_t read_reg(uint8_t reg) {
  switch (reg) {
    case REG_CTRL: {
      critical_section_enter_blocking(&csec_cfg);
      bool run = cfg_run;
      critical_section_exit(&csec_cfg);
      return run ? 1 : 0;
    }
    case REG_MODE: {
      critical_section_enter_blocking(&csec_cfg);
      uint8_t m = cfg_mode;
      critical_section_exit(&csec_cfg);
      return m;
    }
    case REG_CURR: {
      critical_section_enter_blocking(&csec_cfg);
      uint8_t c = cfg_curr;
      critical_section_exit(&csec_cfg);
      return c;
    }
    case REG_TIM: {
      critical_section_enter_blocking(&csec_cfg);
      uint8_t t = (cfg_max_duty_code << 4) | cfg_dur_code;
      critical_section_exit(&csec_cfg);
      return t;
    }
    case REG_RES0: {
      // Reading RES0 resets the WDT and snapshots the result.
      critical_section_enter_blocking(&csec_cfg);
      cfg_wdt_ref = get_absolute_time();
      uint8_t mode = cfg_mode;
      bool detected = cfg_probe_detected;
      critical_section_exit(&csec_cfg);

      // RES0.fault mirrors only the permanent fault (cannot run). A WDT timeout
      // is recoverable and surfaces via the FAULT register's wdt bit instead.
      uint8_t fault_bit = atomic_load(&fault) ? 0x80 : 0x00;

      if (mode == MODE_PROBE) {
        // Result is not cleared by reading in probe mode.
        res1_latch = 0;
        return fault_bit | (detected ? 0x01 : 0x00);
      }

      // Cut mode: snapshot & reset window counts.
      critical_section_enter_blocking(&csec_stat);
      uint32_t n_open = stat_open;
      uint32_t n_short = stat_short;
      uint32_t n_good = stat_good;
      stat_open = stat_short = stat_good = 0;
      critical_section_exit(&csec_stat);

      res1_latch = n_good > 255 ? 255 : (uint8_t)n_good;

      uint32_t n_window = n_open + n_short + n_good;
      uint8_t r_open, r_short;
      if (n_window == 0) {
        r_open = 7;  // no data yet: report fully open
        r_short = 0;
      } else {
        // Floor division keeps r_open + r_short <= 7 (since n_good >= 0).
        r_open = (uint8_t)(n_open * 7 / n_window);
        r_short = (uint8_t)(n_short * 7 / n_window);
      }
      return fault_bit | (r_open << 3) | r_short;
    }
    case REG_RES1:
      return res1_latch;
    case REG_FAULT: {
      critical_section_enter_blocking(&csec_cfg);
      uint8_t wdt_bit = cfg_wdt ? 0x02 : 0x00;
      critical_section_exit(&csec_cfg);
      return wdt_bit | (atomic_load(&fault) ? 0x01 : 0x00);
    }
  }
  return 0;
}

static void i2c_slave_handler(i2c_inst_t* i2c, i2c_slave_event_t event) {
  switch (event) {
    case I2C_SLAVE_RECEIVE:
      if (!i2c_ptr_written) {
        i2c_reg_ptr = i2c_read_byte_raw(i2c);
        i2c_ptr_written = true;
      } else {
        write_reg(i2c_reg_ptr, i2c_read_byte_raw(i2c));
        i2c_reg_ptr++;
      }
      break;
    case I2C_SLAVE_REQUEST:
      i2c_write_byte_raw(i2c, read_reg(i2c_reg_ptr));
      i2c_reg_ptr++;
      break;
    case I2C_SLAVE_FINISH:
      i2c_ptr_written = false;
      break;
    default:
      break;
  }
}

static void init_reg_rw_i2c() {
  gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
  gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
  i2c_init(HOST_I2C, I2C_BAUD);
  i2c_slave_init(HOST_I2C, I2C_DEV_ADDR, &i2c_slave_handler);
}

////////////////////////////////////////////////////////////////////////////////
// Core1: pulse loop.

typedef enum { WIN_OPEN, WIN_SHORT, WIN_GOOD } window_type_t;

// Energize and wait for ignition up to T_IG_MAX_US. Returns the ignition delay
// in us, or -1 if open (no current). HV.EN is left on iff current was detected.
static int64_t wait_ignition() {
  absolute_time_t t0 = get_absolute_time();
  gpio_put(PIN_HV_EN, 1);
  while (!gpio_get(PIN_HV_CURR)) {
    if (absolute_time_diff_us(t0, get_absolute_time()) >= T_IG_MAX_US) {
      gpio_put(PIN_HV_EN, 0);
      return -1;
    }
  }
  return absolute_time_diff_us(t0, get_absolute_time());
}

// One cut-mode window. HV.EN==L && HC.EN==L on entry and exit.
static window_type_t cut_window(uint32_t pulse_dur_us, float max_duty) {
  int64_t tig = wait_ignition();
  if (tig < 0) {
    busy_wait_us(CD_OPEN_US);
    return WIN_OPEN;
  }
  if (tig <= T_IG_SHORT_US) {
    gpio_put(PIN_HV_EN, 0);
    busy_wait_us(CD_SHORT_US);
    return WIN_SHORT;
  }

  // Good pulse: keep HV.EN for PULSE_HANDOVER_US while HC warms up, then hand
  // over to HC for the rest of the pulse. Actual HV pulse shape is set by HV
  // firmware.
  gpio_put(PIN_HC_EN, 1);
  busy_wait_us(PULSE_HANDOVER_US);
  gpio_put(PIN_HV_EN, 0);
  busy_wait_us(pulse_dur_us - PULSE_HANDOVER_US);
  gpio_put(PIN_HC_EN, 0);

  uint32_t cool_duty = (uint32_t)(pulse_dur_us * (1.0f / max_duty - 1.0f));
  busy_wait_us(cool_duty > CD_GOOD_US ? cool_duty : CD_GOOD_US);
  return WIN_GOOD;
}

// One probe-mode window: minimal energize to detect conduction. Returns true
// when current is detected (any ignition before T_IG_MAX_US).
static bool probe_window() {
  int64_t tig = wait_ignition();
  if (tig < 0) {
    busy_wait_us(CD_OPEN_US);
    return false;
  }
  gpio_put(PIN_HV_EN, 0);
  busy_wait_us(CD_SHORT_US);  // de-arc before stopping
  return true;
}

static void error_mode_blink() {
  gpio_put(PIN_HV_EN, 0);
  gpio_put(PIN_HC_EN, 0);
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  while (1) {
    gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
    sleep_ms(LED_ERR_BLINK_ON_MS);
  }
}

static void core1_main() {
  while (true) {
    if (atomic_load(&fault)) {
      goto fatal_error;
    }

    critical_section_enter_blocking(&csec_cfg);
    bool run = cfg_run;
    uint8_t mode = cfg_mode;
    uint8_t dur_code = cfg_dur_code;
    uint8_t max_duty_code = cfg_max_duty_code;
    absolute_time_t wdt_ref = cfg_wdt_ref;
    critical_section_exit(&csec_cfg);

    if (!run) {
      sleep_us(100);
      continue;
    }

    // WDT: host must read RES0 within WDT_TIMEOUT_US. Timeout latches wdt and
    // stops the device.
    if (absolute_time_diff_us(wdt_ref, get_absolute_time()) >= WDT_TIMEOUT_US) {
      critical_section_enter_blocking(&csec_cfg);
      cfg_wdt = true;
      cfg_run = false;
      critical_section_exit(&csec_cfg);
      continue;
    }

    if (mode == MODE_PROBE) {
      if (probe_window()) {
        critical_section_enter_blocking(&csec_cfg);
        cfg_probe_detected = true;
        cfg_run = false;  // probe stops on detection
        critical_section_exit(&csec_cfg);
      }
    } else {
      uint32_t pulse_dur_us = (uint32_t)(dur_code + 1) * 50;
      float max_duty = (float)(max_duty_code + 1) / 16.0f;
      window_type_t w = cut_window(pulse_dur_us, max_duty);

      critical_section_enter_blocking(&csec_stat);
      switch (w) {
        case WIN_OPEN:
          stat_open++;
          break;
        case WIN_SHORT:
          stat_short++;
          break;
        case WIN_GOOD:
          stat_good++;
          break;
      }
      critical_section_exit(&csec_stat);
    }
  }

fatal_error:
  atomic_store(&fault, true);
  gpio_put(PIN_HV_EN, 0);
  gpio_put(PIN_HC_EN, 0);
  while (true) {
    tight_loop_contents();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Core0: main / I2C.

int main() {
  cfg_run = false;
  cfg_wdt = false;
  cfg_probe_detected = false;
  cfg_wdt_ref = nil_time;
  cfg_mode = MODE_ON_RESET;
  cfg_curr = CURR_ON_RESET;
  cfg_max_duty_code = (TIM_ON_RESET >> 4) & 0x0f;
  cfg_dur_code = TIM_ON_RESET & 0x0f;
  critical_section_init(&csec_cfg);
  critical_section_init(&csec_stat);

  gpio_init(PIN_HV_EN);
  gpio_set_dir(PIN_HV_EN, GPIO_OUT);
  gpio_put(PIN_HV_EN, 0);
  gpio_init(PIN_HC_EN);
  gpio_set_dir(PIN_HC_EN, GPIO_OUT);
  gpio_put(PIN_HC_EN, 0);
  gpio_init(PIN_HV_CURR);
  gpio_set_dir(PIN_HV_CURR, GPIO_IN);
  gpio_pull_up(PIN_HV_CURR);

  sleep_ms(1);  // pull-up + comparator settle
  if (gpio_get(PIN_HV_CURR)) {
    error_mode_blink();  // expect L (testboard pulls down)
  }

  init_reg_rw_i2c();
  multicore_launch_core1(core1_main);

  while (true) {
    if (atomic_load(&fault)) {
      error_mode_blink();
    }
    tight_loop_contents();
  }
}
