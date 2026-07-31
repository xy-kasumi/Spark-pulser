// SPDX-License-Identifier: AGPL-3.0-or-later
#define F_CPU 20000000UL

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>

/* -- Pin definitions ------------------------------------------------ */

#define LED_STAT_bm PIN3_bm  // PC3
#define GATE_bm     PIN4_bm  // PA4
#define EN_bm       PIN0_bm  // PC0
#define CURR_bm     PIN1_bm  // PC1
#define FAULT_bm    PIN0_bm  // PB0, active-low

/* -- Pulse configuration -------------------------------------------- */

#define PULSE_DUR_US    10
#define COOLDOWN_DUR_US 100
#define THRESH_MA       200

/* -- Pin initialization --------------------------------------------- */

static void pins_init() {
  // STAT LED PC3: output, initially off
  VPORTC.DIR |= LED_STAT_bm;
  VPORTC.OUT &= ~LED_STAT_bm;

  // !FAULT PB0: output, initially de-asserted (high)
  VPORTB.OUT |= FAULT_bm;
  VPORTB.DIR |= FAULT_bm;

  // GATE PA4: output, initially low
  VPORTA.DIR |= GATE_bm;
  VPORTA.OUT &= ~GATE_bm;

  // EN PC0: input, internal pull-up (default high)
  VPORTC.DIR &= ~EN_bm;
  PORTC.PIN0CTRL = PORT_PULLUPEN_bm;

  // CURR PC1: input, no pull (driven by external comparator)
  VPORTC.DIR &= ~CURR_bm;
}

/* -- STAT LED ------------------------------------------------------- */

static void stat_led(bool on) {
  if (on) {
    VPORTC.OUT |= LED_STAT_bm;
  } else {
    VPORTC.OUT &= ~LED_STAT_bm;
  }
}

/* -- Fault mode ---------------------------------------------------- */

static void fault_mode() __attribute__((noreturn));
static void fault_mode() {
  VPORTA.OUT &= ~GATE_bm;
  VPORTB.OUT &= ~FAULT_bm;
  while (1) {
  }
}

/* -- Elapsed-time timer (pulse & duty timing) ---------------------- */

// Free-running 16-bit counter at CLK_PER (20 MHz) -> 20 ticks/µs.
#define TICKS_PER_US (F_CPU / 1000000UL)

static void timer_init() {
  TCB0.CCMP = 0xFFFF;
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
}

static void timer_reset() {
  TCB0.CNT = 0;
}

// Warning: wraps 3276 µs after timer_reset().
static uint16_t timer_elapsed_ticks() {
  return TCB0.CNT;
}

/* -- DAC initialization (VTH output) ------------------------------- */

static void dac_init() {
  VREF.CTRLA = VREF_DAC0REFSEL_4V34_gc;
  DAC0.DATA = (uint16_t)THRESH_MA * 128 / 434;
  DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm;
}

/* -- Main ----------------------------------------------------------- */

int main() {
  // Run at full 20 MHz: disable prescaler (default is /6)
  CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0;

  pins_init();
  dac_init();
  timer_init();
  _delay_ms(10);  // wait DAC & comparator stabilization

  stat_led(true);

  const uint16_t pulse_ticks = PULSE_DUR_US * TICKS_PER_US;
  const uint16_t cooldown_ticks = COOLDOWN_DUR_US * TICKS_PER_US;

  while (1) {
    // Pre-pulse safety: CURR high without GATE means hardware fault
    if (VPORTC.IN & CURR_bm) {
      fault_mode();
    }

    // Wait for EN rise & activate output
    while (!(VPORTC.IN & EN_bm)) {
    }
    VPORTA.OUT |= GATE_bm;

    // Wait for current to appear, abort if EN drops
    while (!(VPORTC.IN & CURR_bm)) {
      if (!(VPORTC.IN & EN_bm)) {
        VPORTA.OUT &= ~GATE_bm;
        goto wait_en_low;
      }
    }
    timer_reset();  // CURR rise is the reference for both durations

    // Current flowing: hold gate until the pulse elapses or EN drops
    while (timer_elapsed_ticks() < pulse_ticks) {
      if (!(VPORTC.IN & EN_bm)) {
        break;
      }
    }
    VPORTA.OUT &= ~GATE_bm;

    // Thermal duty limit: output stays off for the rest of the window
    while (timer_elapsed_ticks() < cooldown_ticks) {
    }

  wait_en_low:
    while (VPORTC.IN & EN_bm) {
    }
  }
}
