// SPDX-License-Identifier: BSD-2-Clause
#pragma once

#include <stdint.h>

/**
 * Externally observable parameters.
 */
#define LED_ERR_BLINK_OFF_MS 250
#define LED_ERR_BLINK_ON_MS 250

#define I2C_DEV_ADDR 0x3c
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
 * Window timing. See docs/operation.md.
 */
#define T_TRAN_US 3          // ignore period of AC transient current (capacitive coupling)
#define T_IG_SHORT_US 7      // ignition delay <= this -> short window
#define T_IG_MAX_US 500      // no ignition within this -> open window
#define CD_GOOD_US 15        // min cooldown after good window (de-arc)
#define CD_SHORT_US 200      // cooldown after short window; must exceed HV internal cooldown
#define CD_OPEN_US 500       // cooldown after open window
#define PULSE_HANDOVER_US 5  // HV+HC overlap during handover

/**
 * Watchdog: while running, host must read RES0 within this window. Timeout
 * is treated as a fault and stops the device.
 */
#define WDT_TIMEOUT_US 50000  // 50 ms
