// SPDX-License-Identifier: BSD-2-Clause
/**
 * Core0: slow, complex (IRQ allowed) — I2C slave, host-facing register interface.
 * Core1: fast, simple (no IRQ) — pulse loop.
 *
 * Core0 writes to csec_pulse (set by host via I2C); Core1 reads it.
 * Core1 writes to csec_stat; Core0 reads & resets it on CKP_PS read.
 * Both cores can raise error_mode.
 */
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/sync.h"
#include <stdatomic.h>
#include <stdint.h>

// core0, core1 shared. true is sticky.
static _Atomic bool error_mode = false;

// Pulse config: written by core0 from I2C, read by core1 each iteration.
static critical_section_t csec_pulse;
static volatile absolute_time_t csec_pulse_last_ckp;
static volatile uint8_t csec_pulse_pol;
static volatile uint8_t csec_pulse_pcurr;     // stored only; no current control on this rig
static volatile int csec_pulse_pdur;          // pulse duration [us]
static volatile uint8_t csec_pulse_max_duty;  // [%]
static volatile uint8_t csec_pulse_test;      // TEST register byte; stored, not honored

// Pulse statistics: written by core1, read & reset by core0 on CKP_PS read.
static critical_section_t csec_stat;
static uint32_t csec_stat_dur = 0;
static uint32_t csec_stat_dur_pulse = 0;
static uint32_t csec_stat_dur_short = 0;

////////////////////////////////////////////////////////////////////////////////
// Core0: Register read/write & I2C slave.

static const uint8_t REG_POLARITY = 0x01;
static const uint8_t REG_PULSE_CURRENT = 0x02;
static const uint8_t REG_TEMPERATURE = 0x03;
static const uint8_t REG_PULSE_DUR = 0x04;
static const uint8_t REG_MAX_DUTY = 0x05;
static const uint8_t CKP_PS = 0x10;
static const uint8_t REG_TEST = 0x80;

static const uint8_t POL_OFF = 0;
static const uint8_t POL_TPOS = 1;
static const uint8_t POL_TNEG = 2;

static const uint8_t PCURR_ON_RESET = 10;
static const int PDUR_ON_RESET = 500;
static const uint8_t MAX_DUTY_ON_RESET = 25;

// Reported by REG_TEMPERATURE. No real sensor on this rig.
static const uint8_t FAKE_TEMP_C = 25;

static bool i2c_ptr_written = false;
static uint8_t i2c_reg_ptr = 0;

void write_reg(uint8_t reg, uint8_t val) {
  switch (reg) {
  case REG_POLARITY:
    if (val != POL_OFF && val != POL_TPOS && val != POL_TNEG) {
      val = POL_OFF;
    }
    critical_section_enter_blocking(&csec_pulse);
    csec_pulse_pol = val;
    critical_section_exit(&csec_pulse);
    break;
  case REG_PULSE_CURRENT:
    if (val < 1) {
      val = 1;
    } else if (val > 200) {
      val = 200;
    }
    critical_section_enter_blocking(&csec_pulse);
    csec_pulse_pcurr = val;
    critical_section_exit(&csec_pulse);
    break;
  case REG_PULSE_DUR:
    if (val < 5) {
      val = 5;
    } else if (val > 100) {
      val = 100;
    }
    critical_section_enter_blocking(&csec_pulse);
    csec_pulse_pdur = val * 10; // 10us units -> us
    critical_section_exit(&csec_pulse);
    break;
  case REG_MAX_DUTY:
    if (val < 1) {
      val = 1;
    } else if (val > 95) {
      val = 95;
    }
    critical_section_enter_blocking(&csec_pulse);
    csec_pulse_max_duty = val;
    critical_section_exit(&csec_pulse);
    break;
  case REG_TEST:
    critical_section_enter_blocking(&csec_pulse);
    csec_pulse_test = val;
    critical_section_exit(&csec_pulse);
    break;
  }
}

uint8_t read_reg(uint8_t reg) {
  switch (reg) {
  case REG_POLARITY: {
    critical_section_enter_blocking(&csec_pulse);
    uint8_t val = csec_pulse_pol;
    critical_section_exit(&csec_pulse);
    return val;
  }
  case REG_PULSE_CURRENT: {
    critical_section_enter_blocking(&csec_pulse);
    uint8_t val = csec_pulse_pcurr;
    critical_section_exit(&csec_pulse);
    return val;
  }
  case REG_TEMPERATURE:
    return FAKE_TEMP_C;
  case REG_PULSE_DUR: {
    critical_section_enter_blocking(&csec_pulse);
    uint8_t val = csec_pulse_pdur / 10;
    critical_section_exit(&csec_pulse);
    return val;
  }
  case REG_MAX_DUTY: {
    critical_section_enter_blocking(&csec_pulse);
    uint8_t val = csec_pulse_max_duty;
    critical_section_exit(&csec_pulse);
    return val;
  }
  case CKP_PS: {
    absolute_time_t now = get_absolute_time();
    critical_section_enter_blocking(&csec_pulse);
    csec_pulse_last_ckp = now;
    critical_section_exit(&csec_pulse);

    critical_section_enter_blocking(&csec_stat);
    uint32_t stat_dur = csec_stat_dur;
    uint32_t stat_dur_pulse = csec_stat_dur_pulse;
    uint32_t stat_dur_short = csec_stat_dur_short;
    csec_stat_dur = 0;
    csec_stat_dur_pulse = 0;
    csec_stat_dur_short = 0;
    critical_section_exit(&csec_stat);

    if (stat_dur == 0) {
      return 0;
    }
    uint8_t visible_r_pulse = (uint64_t)stat_dur_pulse * 15 / stat_dur;
    uint8_t visible_r_short = (uint64_t)stat_dur_short * 15 / stat_dur;
    return (visible_r_pulse & 0xf) << 4 | (visible_r_short & 0xf);
  }
  case REG_TEST: {
    critical_section_enter_blocking(&csec_pulse);
    uint8_t val = csec_pulse_test;
    critical_section_exit(&csec_pulse);
    return val;
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

// Stop pulsing if host hasn't polled CKP_PS within this window.
static const uint32_t CKP_TIMEOUT_US = 100000; // 100 ms

typedef struct {
  uint32_t total_us;
  uint32_t pulse_us; // pulse_duration_us on normal pulse, else 0
  uint32_t short_us; // SHORT_COOLDOWN_US on short, else 0
} pulse_result_t;

// HV:EN==L && HC:EN==L on entry.
static pulse_result_t single_pulse(uint32_t pulse_duration_us, float duty) {
  uint32_t duty_cooldown_us = (uint32_t)(pulse_duration_us * (1.0f / duty - 1.0f));
  uint32_t duty_limit_us = duty_cooldown_us > PULSE_COOLDOWN_US
                               ? duty_cooldown_us
                               : PULSE_COOLDOWN_US;

  absolute_time_t t0 = get_absolute_time();
  gpio_put(PIN_HV_EN, 1);
  while (!gpio_get(PIN_HV_CURR)) {
    tight_loop_contents();
  }
  int64_t wait_us = absolute_time_diff_us(t0, get_absolute_time());

  pulse_result_t r;
  if (wait_us < TOO_SMALL_US) {
    gpio_put(PIN_HV_EN, 0);
    busy_wait_us(SHORT_COOLDOWN_US);
    r.total_us = (uint32_t)wait_us + SHORT_COOLDOWN_US;
    r.pulse_us = 0;
    r.short_us = SHORT_COOLDOWN_US;
  } else {
    // keep HV.EN for PULSE_HANDOVER_US while HC warms up.
    // Actual HV pulse is determined by HV firmware.
    gpio_put(PIN_HC_EN, 1);
    busy_wait_us(PULSE_HANDOVER_US);

    // Switch to HC-only region (main pulse)
    gpio_put(PIN_HV_EN, 0);
    busy_wait_us(pulse_duration_us - PULSE_HANDOVER_US);

    // cooldown
    gpio_put(PIN_HC_EN, 0);
    busy_wait_us(duty_limit_us);

    r.total_us = (uint32_t)wait_us + pulse_duration_us + duty_limit_us;
    r.pulse_us = pulse_duration_us;
    r.short_us = 0;
  }
  return r;
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
    if (atomic_load(&error_mode)) {
      goto fatal_error;
    }

    critical_section_enter_blocking(&csec_pulse);
    absolute_time_t last_ckp = csec_pulse_last_ckp;
    uint8_t pol = csec_pulse_pol;
    int pdur = csec_pulse_pdur;
    uint8_t max_duty = csec_pulse_max_duty;
    critical_section_exit(&csec_pulse);

    bool ckp_timeout =
        absolute_time_diff_us(last_ckp, get_absolute_time()) >= CKP_TIMEOUT_US;
    if (ckp_timeout || pol == POL_OFF) {
      sleep_us(100);
      continue;
    }

    pulse_result_t r = single_pulse((uint32_t)pdur, (float)max_duty / 100.0f);

    critical_section_enter_blocking(&csec_stat);
    csec_stat_dur += r.total_us;
    csec_stat_dur_pulse += r.pulse_us;
    csec_stat_dur_short += r.short_us;
    critical_section_exit(&csec_stat);
  }

fatal_error:
  atomic_store(&error_mode, true);
  gpio_put(PIN_HV_EN, 0);
  gpio_put(PIN_HC_EN, 0);
  while (true) {
    tight_loop_contents();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Core0: main / I2C.

int main() {
  csec_pulse_last_ckp = nil_time;
  csec_pulse_pol = POL_OFF;
  csec_pulse_pcurr = PCURR_ON_RESET;
  csec_pulse_pdur = PDUR_ON_RESET;
  csec_pulse_max_duty = MAX_DUTY_ON_RESET;
  csec_pulse_test = 0;
  critical_section_init(&csec_pulse);
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

  sleep_ms(1); // pull-up + comparator settle
  if (gpio_get(PIN_HV_CURR)) {
    error_mode_blink(); // expect L (testboard pulls down)
  }

  init_reg_rw_i2c();
  multicore_launch_core1(core1_main);

  while (true) {
    if (atomic_load(&error_mode)) {
      error_mode_blink();
    }
    tight_loop_contents();
  }
}
