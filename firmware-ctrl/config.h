// SPDX-License-Identifier: BSD-2-Clause
#pragma once

#include <stdint.h>

/**
 * ATtiny1616 controller firmware for the PULSER ctrl board.
 *
 * Same device spec as firmware-test-integ (Pico 2): see docs/i2c-registers.md
 * (I2C register map) and docs/operation.md (window/timing model). Ported to a
 * single core — the pulse loop runs in the foreground and the I2C slave runs in
 * the TWI ISR (see main.c).
 */

/**
 * I2C host interface.
 */
#define I2C_DEV_ADDR 0x3c

/**
 * GPIO pin assignments (ctrl board, schematic "Draft ctrl r2"). The schematic
 * is temporary, so the mapping lives here for easy retargeting.
 *
 * HV/HC signals are all on PORTA. I2C uses the TWI0 default pins PB0=SCL,
 * PB1=SDA (routed automatically when TWI0 is enabled; no PORTMUX needed).
 */
#define HV_EN_bm    PIN1_bm  // PA1, output — HV gate enable, active H
#define HV_CURR_bm  PIN2_bm  // PA2, input  — comparator out, idle L, H = current detected
#define HV_FAULT_bm PIN3_bm  // PA3, input  — HV board fault, active L; latches permanent fault
#define HC_EN_bm    PIN4_bm  // PA4, output — HC gate enable, active H
#define HC_CURR_bm  PIN5_bm  // PA5, output — reserved, unused by spec logic

/**
 * Window timing. See docs/operation.md. Values match firmware-test-integ.
 */
#define T_TRAN_US 3          // ignore period of AC transient current (capacitive coupling)
#define T_IG_SHORT_US 7      // ignition delay <= this -> short window
#define T_IG_MAX_US 500      // no ignition within this -> open window
#define CD_GOOD_US 15        // min cooldown after good window (de-arc)
#define CD_SHORT_US 100      // cooldown after short window; must exceed HV internal cooldown
#define CD_OPEN_US 500       // cooldown after open window
#define PULSE_HANDOVER_US 5  // HV+HC overlap during handover

/**
 * Watchdog: while running, host must read RES0 within this window. Timeout
 * is treated as a recoverable fault and stops the device.
 */
#define WDT_TIMEOUT_US 50000  // 50 ms

/**
 * Derived timer-tick constants.
 *
 * Fine timer TCB0 runs at CLK_PER = 20 MHz => 50 ns/tick, so ticks = us * 20.
 * The ignition-detect loop compares against these precomputed thresholds to
 * stay free of multiply/divide. A single window never exceeds the 16-bit range
 * (~3.3 ms), so elapsed time is a wrap-safe uint16 subtraction.
 */
#define TICKS_PER_US 20u
#define T_TRAN_TICKS     (T_TRAN_US * TICKS_PER_US)      // 60
#define T_IG_SHORT_TICKS (T_IG_SHORT_US * TICKS_PER_US)  // 140
#define T_IG_MAX_TICKS   (T_IG_MAX_US * TICKS_PER_US)    // 10000
#define PROBE_CHECK_TICKS (100u * TICKS_PER_US)          // 2000  (probe latch phase)
#define PROBE_CYCLE_TICKS (500u * TICKS_PER_US)          // 10000 (probe total cycle)

/**
 * Coarse timer TCA0 runs at CLK_PER / 1024 = 51.2 us/tick, used as the software
 * WDT clock. 50 ms / 51.2 us = 976.6 -> 977 ticks. uint16 wrap-safe (period is
 * ~3.35 s, far longer than the 50 ms timeout).
 */
#define WDT_TIMEOUT_TICKS 977u
