// SPDX-License-Identifier: BSD-2-Clause
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// Pin assignments.
#define PIN_HV_CURR 0  // input  — comparator out, idle L, H = current detected
#define PIN_HV_EN 1    // output — HV gate enable, active H
#define PIN_HC_EN 2    // output — HC gate enable, active H

// Timing (microseconds).
#define TOO_SMALL_US 5         // CURR rises faster than this -> short
#define SHORT_COOLDOWN_US 100  // post-short cooldown
#define PULSE_COOLDOWN_US 20   // minimum dead time between pulses
#define PULSE_HANDOVER_US 5
#define PULSE_DUR_US 100       // HC on-time passed to single_pulse
#define MAX_DUTY 0.5f          // upper bound on HC on-time / period

static void error_mode() {
  gpio_put(PIN_HV_EN, 0);
  gpio_put(PIN_HC_EN, 0);
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  while (1) {
    gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
    sleep_ms(100);
  }
}

// HV:EN==L && HC:EN==L on entry. Current/mux config is external.
static void single_pulse(uint32_t pulse_duration_us, float duty) {
  uint32_t duty_cooldown_us = (uint32_t)(pulse_duration_us * (1.0f / duty - 1.0f));
  uint32_t duty_limit_us = duty_cooldown_us > PULSE_COOLDOWN_US
                               ? duty_cooldown_us
                               : PULSE_COOLDOWN_US;

  gpio_put(PIN_HV_EN, 1);
  absolute_time_t t0 = get_absolute_time();
  while (!gpio_get(PIN_HV_CURR)) {
    tight_loop_contents();
  }
  int64_t wait_us = absolute_time_diff_us(t0, get_absolute_time());

  if (wait_us < TOO_SMALL_US) {
    gpio_put(PIN_HV_EN, 0);
    busy_wait_us(SHORT_COOLDOWN_US);
  } else {
    // keep HV.EN for PULSE_HANDOVER_US while HC warms up
    // Actual HV pulse is determined by HV firmware.
    gpio_put(PIN_HC_EN, 1);
    busy_wait_us(PULSE_HANDOVER_US);

    // Switch to HC-only region (main pulse)
    gpio_put(PIN_HV_EN, 0);
    busy_wait_us(pulse_duration_us - PULSE_HANDOVER_US);

    // cooldown
    gpio_put(PIN_HC_EN, 0);
    busy_wait_us(duty_limit_us);
  }
}

int main() {
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
    error_mode();  // expect L (testboard pulls down)
  }

  while (1) {
    single_pulse(PULSE_DUR_US, MAX_DUTY);
  }
}
