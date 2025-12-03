// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * To get reliable GATE/DETECT handling, we use dual core.
 * Pin control responsibility is cleanly separated.
 *
 * Once Core0 finishes initialization and passed self-check,
 * Core1 is launched and Core0 relinquishes control over these pins.
 *
 * Core0: slow, complex (IRQ, float-compute allowed)
 * - PIN_I2C_SDA, PIN_I2C_SCL
 * - PIN_TS_I2C_SDA, PIN_TS_I2C_SCL
 * - PIN_LED_STATUS
 *
 * Core1: fast, simple (no IRQ)
 * - PIN_GATE_IG, PIN_GATE_MAIN_PWM
 * - PIN_CURR_DETECT
 * - PIN_MUX_V0, PIN_MUX_VC
 *
 * Core0 and Core1 share global error_mode flag. Both core can raise error.
 * Core0 write to a critical section csec_pulse, and Core1 read it.
 * No Core1->Core0 data flow (other than global error).
 */
#include "config.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/i2c_slave.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include <inttypes.h>
#include <math.h>
#include <stdatomic.h>

// core0, core1 shared
static _Atomic bool error_mode =
    false; // reading & writing true is allowed. never write false.

// core0, core1 shared, only accessed in csec_pulse section or initialization.
static critical_section_t csec_pulse;
static volatile absolute_time_t csec_pulse_last_ckp;
static volatile uint8_t csec_pulse_pol;
static volatile uint8_t csec_pulse_pcurr;
static volatile int csec_pulse_pdur;
static volatile uint8_t csec_pulse_max_duty;
static volatile bool csec_pulse_test_disable_short;
static volatile bool csec_pulse_test_disable_ig_wait;

// core0, core1 shared, only accessed in csec_stat section.
static critical_section_t csec_stat;
static uint32_t csec_stat_dur = 0;
static uint32_t csec_stat_dur_pulse = 0;
static uint32_t csec_stat_dur_short = 0;

////////////////////////////////////////////////////////////////////////////////
// Core1: Gate & current threshold PWM driving and computation.

static const uint8_t PCURR_ON_RESET = 10;    // 1A
static const int PDUR_ON_RESET = 500;        // 500us
static const uint8_t MAX_DUTY_ON_RESET = 25; // 25%

static const uint16_t PWM_GATE_NUM_CYCLE = 300; // create 500kHz (=150MHz/300)

// Maximum duty ratio: empirtically determined to drive dummy load.
static const float PWM_MAX_DUTY = 0.95;

// Maximum allowed current ripple assuming (low-ish) 15V gap voltage & largest
// duty. If (current measurement) > (target current) + MAX_CURR_OVERSHOOT_MA, it
// means the gap is probably in short-mode or some unexpected thing is
// happening. Even in normal situation, output current will fluctuate between
// (target current) -/+ MAX_CURR_OVERSHOOT_MA. (36V - 15V) / 10uH * 2us * 70%
// (PWM_MAX_DUTY) = 2.94A
// 2.94A x 1.2 (to prevent premature shutdown)
static const float MAX_CURR_OVERSHOOT_A = 3.58;

void init_out_pwm() {
  gpio_set_function(PIN_GATE_MAIN_PWM, GPIO_FUNC_PWM);
  pwm_set_wrap(PWM_GATE_MAIN_PWM, PWM_GATE_NUM_CYCLE - 1);
  pwm_set_chan_level(PWM_GATE_MAIN_PWM, PWM_CHAN_GATE_MAIN_PWM, 0);
  pwm_set_enabled(PWM_GATE_MAIN_PWM, true);
}

// Sets current output duty factor.
static void set_out_level(float duty) {
  int level = roundf(duty * PWM_GATE_NUM_CYCLE);
  if (level < 0) {
    level = 0;
  } else if (level > PWM_GATE_NUM_CYCLE) {
    level = PWM_GATE_NUM_CYCLE;
  }
  pwm_set_chan_level(PWM_GATE_MAIN_PWM, PWM_CHAN_GATE_MAIN_PWM, level);
}

static void turnoff_out() {
  set_out_level(0);
  gpio_put(PIN_GATE_IG, false);
}

////////////////////////////////////////////////////////////////////////////////
// Core1: Current detection ADC.

static volatile float latest_curr_a = 0;

void init_curr_detect() {
  adc_init();
  adc_gpio_init(PIN_CURR_DETECT);
  adc_select_input(ADC_CURR_DETECT);

  // Change to PWM mode to reduce power ripple.
  gpio_init(PIN_POWER_PS);
  gpio_set_dir(PIN_POWER_PS, true);
  gpio_put(PIN_POWER_PS, true);

  // wait for power to stabilize
  sleep_ms(10);

  // start sampling
  hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
}

// This can only measure up to max 22.5A, and with 5.5mA resolution.
// Also, actual updates will happen at about 500kHz.
static float get_latest_current_a() {
  if (adc_hw->cs & ADC_CS_READY_BITS) {
    // 12bit, 3.0V max, 133mV/A.
    // 1 LSB = 0.732mV
    // max measurement: 18.79A

    uint16_t val = (uint16_t)adc_hw->result;
    float voltage_mv = (float)val * 0.732;
    latest_curr_a = (voltage_mv - 500) * (1.0 / 133);

    // start next sampling
    hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
  }
  return latest_curr_a;
}

////////////////////////////////////////////////////////////////////////////////
// Core0: Temperature sensor

static const int TEMP_INVALID = -1000;
static int current_temp_c = TEMP_INVALID;

// Update current_temp by doing ADC read.
// Returns true if ok, false otherwise.
bool update_temp_blocking() {
  uint8_t data = 0;
  if (i2c_write_timeout_us(TS_I2C, TS_I2C_DEV_ADDR, &data, 1, true,
                           TS_I2C_TIMEOUT_US) != 1) {
    return false;
  }
  if (i2c_read_timeout_us(TS_I2C, TS_I2C_DEV_ADDR, &data, 1, false,
                          TS_I2C_TIMEOUT_US) != 1) {
    return false;
  }

  current_temp_c = (int8_t)data;
  return true;
}

// returns: true if ok, false if bad.
bool init_temp_sensor_and_check_sanity() {
  gpio_set_function(PIN_TS_I2C_SCL, GPIO_FUNC_I2C);
  gpio_set_function(PIN_TS_I2C_SDA, GPIO_FUNC_I2C);
  i2c_init(TS_I2C, I2C_BAUD);

  if (!update_temp_blocking()) {
    return false;
  }
  if (current_temp_c == TEMP_INVALID || current_temp_c > MAX_ALLOWED_TEMP) {
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Core0: Register read/write & I2C

static const uint8_t REG_POLARITY = 0x01;
static const uint8_t REG_PULSE_CURRENT = 0x02;
static const uint8_t REG_TEMPERATURE = 0x03;
static const uint8_t REG_PULSE_DUR = 0x04;
static const uint8_t REG_MAX_DUTY = 0x05;
static const uint8_t CKP_PS = 0x10;
static const uint8_t REG_TEST = 0x80;

static const uint8_t POL_OFF = 0;
static const uint8_t POL_TPOS = 1; // Tool+
static const uint8_t POL_TNEG = 2; // Tool-

static const uint8_t TEST_DISABLE_SHORT = 1 << 0;
static const uint8_t TEST_DISABLE_IG_WAIT = 1 << 1;

static bool i2c_ptr_written = false;
static uint8_t i2c_reg_ptr = 0;

// write to register. ignores invalid reg address.
void write_reg(uint8_t reg, uint8_t val) {
  switch (reg) {
  case REG_POLARITY:
    if (val != POL_OFF && val != POL_TPOS && val != POL_TNEG) {
      val = POL_OFF; // treat unknown value as OFF for safety.
    }
    critical_section_enter_blocking(&csec_pulse);
    csec_pulse_pol = val;
    critical_section_exit(&csec_pulse);
    break;
  case REG_PULSE_CURRENT:
    // Truncate to valid range.
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
    // Truncate to valid range.
    if (val < 5) {
      val = 5;
    } else if (val > 100) {
      val = 100;
    }
    critical_section_enter_blocking(&csec_pulse);
    csec_pulse_pdur = val * 10; // convert to us
    critical_section_exit(&csec_pulse);
    break;
  case REG_MAX_DUTY:
    // Truncate to valid range.
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
    csec_pulse_test_disable_short = val & TEST_DISABLE_SHORT;
    csec_pulse_test_disable_ig_wait = val & TEST_DISABLE_IG_WAIT;
    critical_section_exit(&csec_pulse);
    break;
  }
}

// read register. return0 for invalid reg address.
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
    // Temp sensor is initialized before I2C.
    if (current_temp_c < 0) {
      return 0;
    } else if (current_temp_c >= 255) {
      return 255;
    } else {
      return current_temp_c;
    }
  case REG_PULSE_DUR: {
    critical_section_enter_blocking(&csec_pulse);
    uint8_t val = csec_pulse_pdur / 10; // convert from us to 10us unit
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

    // Move content to visible buffer and reset internal buffer.
    // Copy to temp vars to leave critical section ASAP to avoid distrupting
    // core1.
    critical_section_enter_blocking(&csec_stat);
    uint32_t stat_dur = csec_stat_dur;
    uint32_t stat_dur_pulse = csec_stat_dur_pulse;
    uint32_t stat_dur_short = csec_stat_dur_short;
    // reset
    csec_stat_dur = 0;
    csec_stat_dur_pulse = 0;
    csec_stat_dur_short = 0;
    critical_section_exit(&csec_stat);

    // convert
    if (stat_dur == 0) {
      // was not active
      return 0;
    } else {
      uint8_t visible_r_pulse = (uint64_t)stat_dur_pulse * 15 / stat_dur;
      uint8_t visible_r_short = (uint64_t)stat_dur_short * 15 / stat_dur;
      return (visible_r_pulse & 0xf) << 4 | (visible_r_short & 0xf);
    }
  }
  case REG_TEST: {
    critical_section_enter_blocking(&csec_pulse);
    uint8_t val = 0;
    if (csec_pulse_test_disable_short) {
      val |= TEST_DISABLE_SHORT;
    }
    if (csec_pulse_test_disable_ig_wait) {
      val |= TEST_DISABLE_IG_WAIT;
    }
    critical_section_exit(&csec_pulse);
    return val;
  }
  }
  return 0;
}

// handle I2C requests.
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

void init_reg_rw_i2c() {
  gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
  gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
  i2c_init(HOST_I2C, I2C_BAUD);
  i2c_slave_init(HOST_I2C, I2C_DEV_ADDR, &i2c_slave_handler);
}

////////////////////////////////////////////////////////////////////////////////
// Main loops for each core.

static const uint32_t CKP_TIMEOUT_US = 100000; // 100ms

// Treat ignition time of this or smaller as short.
static const uint16_t IG_THRESH_SHORT_US = 5;
// Treat ignition time of this or larger as open.
static const uint16_t IG_THRESH_OPEN_US = 1000;

static const uint16_t COOLDOWN_SHORT_US = 100;
static const uint16_t COOLDOWN_OPEN_US = 50;

static const uint16_t COOLDOWN_PULSE_MIN_US = 5;

static const uint32_t MASK_ALL_MUX_PINS = (1 << PIN_MUX_V0) | (1 << PIN_MUX_VC);

typedef enum {
  // Ignition didn't happen within specified time window.
  IG_RESULT_OPEN,

  // Ignition happened too early; considered as short.
  IG_RESULT_SHORT,

  // Ignition happened normally; should proceed to pulse current control.
  IG_RESULT_OK,
} ig_result_t;

static inline float clampf(float val, float min, float max) {
  if (val < min) {
    return min;
  } else if (val > max) {
    return max;
  } else {
    return val;
  }
}

static inline int maxi(int a, int b) { return (a > b) ? a : b; }

/**
 * Set polarity.
 * Takes at most 10us.
 */
void set_pol(uint8_t pol) {
  sleep_us(DISCHARGE_MAX_SETTLE_TIME_US);

  gpio_put_masked(MASK_ALL_MUX_PINS, 0);
  sleep_us(MUX_MAX_SETTLE_TIME_US);

  if (pol == POL_TPOS) {
    // T+
    gpio_put_masked(MASK_ALL_MUX_PINS, 1 << PIN_MUX_VC);
  } else {
    // T- (including OFF)
    gpio_put_masked(MASK_ALL_MUX_PINS, 1 << PIN_MUX_V0);
  }
  sleep_us(MUX_MAX_SETTLE_TIME_US);
}

/**
 * Start applying HV to the gap and wait for discharge to happen until
 * IG_THRESH_OPEN_US passes.
 */
ig_result_t ignite(bool test_disable_short, bool test_disable_ig_wait) {
  gpio_put(PIN_GATE_IG, true);
  if (test_disable_ig_wait) {
    return IG_RESULT_OK;
  }

  for (int igt_us = 0; igt_us < IG_THRESH_OPEN_US; igt_us++) {
    // This threshold is important.
    // It must be smaller than saturated output current of 100V converter.
    // The current is determined by: (36V - D7.Vf) / R2
    //
    // If the threshold is too big, ignition can't be detected and 100V
    // circuit will burn.
    if (get_latest_current_a() >= 1.0f) {
      if (igt_us <= IG_THRESH_SHORT_US && !test_disable_short) {
        return IG_RESULT_SHORT;
      } else {
        return IG_RESULT_OK;
      }
    }
    sleep_us(1);
    critical_section_enter_blocking(&csec_stat);
    csec_stat_dur++;
    critical_section_exit(&csec_stat);
  }
  return IG_RESULT_OPEN;
}

void core1_main() {
  uint8_t curr_pol = POL_OFF;
  float curr_pcurr_a = 1;

  while (true) {
    // Exit if global error mode is set.
    if (atomic_load(&error_mode)) {
      goto fatal_error;
    }

    // Fetch latest pulse config for processing.
    critical_section_enter_blocking(&csec_pulse);
    absolute_time_t last_ckp = csec_pulse_last_ckp;
    uint8_t new_pol = csec_pulse_pol;
    uint8_t new_pcurr = csec_pulse_pcurr;
    uint16_t pdur = csec_pulse_pdur;
    uint8_t max_duty = csec_pulse_max_duty;
    bool test_disable_short = csec_pulse_test_disable_short;
    bool test_disable_ig_wait = csec_pulse_test_disable_ig_wait;
    critical_section_exit(&csec_pulse);
    int pinterval = (uint32_t)pdur * 100 / (uint32_t)max_duty;
    int pcooldown = pinterval - pdur;
    float new_pcurr_a = (float)new_pcurr * 0.1f;

    // Apply config changes.
    if (curr_pol != new_pol) {
      set_pol(new_pol);
      curr_pol = new_pol;
    }
    curr_pcurr_a = new_pcurr_a;

    // Activeness check
    bool ckp_timeout =
        absolute_time_diff_us(last_ckp, get_absolute_time()) >= CKP_TIMEOUT_US;
    if (ckp_timeout || curr_pol == POL_OFF) {
      continue;
    }

    /////
    //  Pulse control.

    absolute_time_t t_ig_begin = get_absolute_time();
    ig_result_t ig_res = ignite(test_disable_short, test_disable_ig_wait);
    int dt_ig = absolute_time_diff_us(t_ig_begin, get_absolute_time());

    if (ig_res == IG_RESULT_SHORT) {
      // short; turn-off and short-cooldown.
      turnoff_out();
      sleep_us(COOLDOWN_SHORT_US);
      critical_section_enter_blocking(&csec_stat);
      csec_stat_dur += COOLDOWN_SHORT_US;
      csec_stat_dur_short += COOLDOWN_SHORT_US;
      critical_section_exit(&csec_stat);
    } else if (ig_res == IG_RESULT_OPEN) {
      // open
      turnoff_out();
      // cooldown is necessary to recharge boostrap capacitor of the gate driver
      // for ignition power supply
      sleep_us(COOLDOWN_OPEN_US);
      critical_section_enter_blocking(&csec_stat);
      csec_stat_dur += COOLDOWN_OPEN_US;
      critical_section_exit(&csec_stat);
    } else {
      // normal pulse; update stats & wait for pulse duration.
      // control current for pdur.

      // "neutral" duty for 20V gap is 0.55. Set conservative and rely
      // on I-control to adjust.
      const float duty_neutral = 0.5;
      const float gain = 0.02;
      const float t_integ = 20e-6;
      float err_accum = 0;

      // immediately start CC PSU before C5 runs out and/or GATE_IG turns off.
      set_out_level(duty_neutral);

      for (int us = 0; us < pdur; us++) {
        // Turn off ignition voltage and hand over to PWM current.
        // Wait for a us to have some overlap.
        if (us >= 1) {
          gpio_put(PIN_GATE_IG, false);
        }

        // Start current control after ignition effect settles.
        if (us >= 15) {
          // Control constant-current PWM.
          float curr_a = get_latest_current_a();
          if (curr_a > curr_pcurr_a + MAX_CURR_OVERSHOOT_A) {
            // current is too high for a normal gap.
            // gap might get shorted during the pulse,
            // or control is oscillating.
            // NOTE: maybe this should be counted and should be exposed via
            // register?
            break;
          }
          float err = (curr_pcurr_a - curr_a);
          err_accum += err * 1e-6;

          float duty = duty_neutral + err * gain + err_accum * (gain / t_integ);
          duty = clampf(duty, 0, PWM_MAX_DUTY);
          set_out_level(duty);
        } else {
          set_out_level(duty_neutral);
        }
        sleep_us(1);
      }
      // Turn-off
      turnoff_out();

      // Record spent time as pulse time.
      critical_section_enter_blocking(&csec_stat);
      csec_stat_dur += pdur;
      csec_stat_dur_pulse += pdur;
      critical_section_exit(&csec_stat);

      // Cooldown to stay within max duty.
      // Ignition wait time can be counted as cooldown.
      int cooldown_time = maxi(pcooldown - dt_ig, COOLDOWN_PULSE_MIN_US);
      sleep_us(cooldown_time);

      // do not count cooldown of succesful pulse as either short, pulse, open.
    }
  }

fatal_error:
  atomic_store(&error_mode, true); // tell other core

  // De-energize safely.
  set_out_level(0);
  sleep_us(DISCHARGE_MAX_SETTLE_TIME_US);
  gpio_put_masked(MASK_ALL_MUX_PINS, 0);
  sleep_ms(MUX_MAX_SETTLE_TIME_US);

  while (true) {
  }
}

// core0
int main() {
  // Init compute.
  csec_pulse_last_ckp = 0;
  csec_pulse_pol = POL_OFF;
  csec_pulse_pcurr = PCURR_ON_RESET;
  csec_pulse_pdur = PDUR_ON_RESET;
  csec_pulse_max_duty = MAX_DUTY_ON_RESET;
  csec_pulse_test_disable_short = false;
  critical_section_init(&csec_pulse);
  critical_section_init(&csec_stat);

  // Init I/O to safe state.
  const uint32_t output_mask = (1 << PIN_LED_STATUS) | (1 << PIN_I2C_SDA) |
                               (1 << PIN_I2C_SCL) | (1 << PIN_TS_I2C_SDA) |
                               (1 << PIN_TS_I2C_SCL) | (1 << PIN_GATE_IG) |
                               (1 << PIN_GATE_MAIN_PWM) | MASK_ALL_MUX_PINS;

  gpio_init_mask(output_mask);
  gpio_set_dir_masked(output_mask, output_mask);
  gpio_clr_mask(output_mask);

  ////////////////////////////////////////////////////////////
  // Initialize "modules" with sanity checks, starting from safe ones.

  if (!init_temp_sensor_and_check_sanity()) {
    goto fatal_error;
  }

  init_curr_detect();
  init_out_pwm();
  init_reg_rw_i2c();
  multicore_launch_core1(core1_main);

  ////////////////////////////////////////////////////////////
  // device is now operating normally.
  gpio_put(PIN_LED_STATUS, 1);

  // main loop
  while (true) {
    if (atomic_load(&error_mode)) {
      goto fatal_error;
    }

    update_temp_blocking();
    if (current_temp_c == TEMP_INVALID || current_temp_c > MAX_ALLOWED_TEMP) {
      goto fatal_error;
    }
  }

fatal_error:
  atomic_store(&error_mode, true); // tell other core

  while (true) {
    // "error blink" forever
    gpio_put(PIN_LED_STATUS, 1);
    sleep_ms(LED_ERR_BLINK_ON_MS);
    gpio_put(PIN_LED_STATUS, 0);
    sleep_ms(LED_ERR_BLINK_OFF_MS);
  }
}
