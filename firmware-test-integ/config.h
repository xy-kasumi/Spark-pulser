// SPDX-License-Identifier: BSD-2-Clause
#pragma once

#include <stdint.h>

/**
 * Externally observable parameters.
 */
#define LED_ERR_BLINK_OFF_MS 250
#define LED_ERR_BLINK_ON_MS 250

#define I2C_DEV_ADDR 0x3b
#define I2C_BAUD 400000

/**
 * GPIO pin assignments.
 *
 * GP0/1/2 drive the test rig (HV/HC). GP16/17 expose the I2C slave
 * to a host MCU; these are I2C0 alternate pins on RP2350, so the
 * peripheral stays i2c0 (same as firmware/).
 */
#define PIN_HV_CURR 0    // input — comparator out, idle L, H = current detected
#define PIN_HV_EN 1      // output — HV gate enable, active H
#define PIN_HC_EN 2      // output — HC gate enable, active H
#define PIN_I2C_SDA 16   // I2C0 SDA
#define PIN_I2C_SCL 17   // I2C0 SCL

#define HOST_I2C i2c0

/**
 * Pulse timing (microseconds).
 */
#define TOO_SMALL_US 5         // CURR rises faster than this -> short
#define SHORT_COOLDOWN_US 200  // post-short cooldown; must exceed HV internal cooldown
#define PULSE_COOLDOWN_US 15   // minimum dead time between pulses
#define PULSE_HANDOVER_US 5    // HV+HC overlap during handover
