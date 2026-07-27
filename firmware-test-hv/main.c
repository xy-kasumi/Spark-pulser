#define F_CPU 20000000UL

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include <util/delay_basic.h>

/* ── Pin definitions ──────────────────────────────────────────────── */

#define LED_STAT_bm     PIN3_bm   // PC3
#define GATE_bm         PIN4_bm   // PA4
#define EN_bm           PIN0_bm   // PC0
#define CURR_bm         PIN1_bm   // PC1

/* ── Pulse configuration ──────────────────────────────────────────── */

#define PULSE_DUR_US       10
#define COOLDOWN_DUR_US    100
#define THRESH_MA          200

/* ── Pin initialization ───────────────────────────────────────────── */

static void pins_init() {
    // STAT LED PC3: output, initially off
    VPORTC.DIR |= LED_STAT_bm;
    VPORTC.OUT &= ~LED_STAT_bm;

    // GATE PA4: output, initially low
    VPORTA.DIR |= GATE_bm;
    VPORTA.OUT &= ~GATE_bm;

    // EN PC0: input, internal pull-up (default high)
    VPORTC.DIR &= ~EN_bm;
    PORTC.PIN0CTRL = PORT_PULLUPEN_bm;

    // CURR PC1: input, no pull (driven by external comparator)
    VPORTC.DIR &= ~CURR_bm;
}

/* ── STAT LED ─────────────────────────────────────────────────────── */

static void stat_led(bool on) {
    if (on)
        VPORTC.OUT |=  LED_STAT_bm;
    else
        VPORTC.OUT &= ~LED_STAT_bm;
}

/* ── Error mode ──────────────────────────────────────────────────── */

static void error_mode() __attribute__((noreturn));
static void error_mode() {
    VPORTA.OUT &= ~GATE_bm;
    while (1) {
        VPORTC.OUT ^= LED_STAT_bm;
        _delay_ms(100);
    }
}

/* ── DAC initialization (VTH output) ─────────────────────────────── */

static void dac_init() {
    VREF.CTRLA = VREF_DAC0REFSEL_4V34_gc;
    DAC0.DATA  = (uint16_t)THRESH_MA * 128 / 434;
    DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm;
}

/* ── Main ─────────────────────────────────────────────────────────── */

int main() {
    // Run at full 20 MHz: disable prescaler (default is /6)
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0;

    pins_init();
    dac_init();
    _delay_ms(10); // wait DAC & comparator stabilization

    stat_led(true);

    // At 20 MHz, _delay_loop_2 = 4 cycles/iter = 0.2 µs → ticks = µs × 5
    const uint16_t pulse_ticks    = PULSE_DUR_US * 5;
    const uint16_t cooldown_ticks = COOLDOWN_DUR_US * 5;

    while (1) {
        // Pre-pulse safety: CURR high without GATE means hardware fault
        if (VPORTC.IN & CURR_bm) {
            error_mode();
        }

        // Wait for EN to go high
        while (!(VPORTC.IN & EN_bm)) {
        }

        VPORTA.OUT |= GATE_bm;

        // Wait for current to appear, abort if EN drops
        while (!(VPORTC.IN & CURR_bm)) {
            if (!(VPORTC.IN & EN_bm)) {
                VPORTA.OUT &= ~GATE_bm;
                goto cooldown;
            }
        }

        // Current flowing — hard-timed pulse
        _delay_loop_2(pulse_ticks);

        VPORTA.OUT &= ~GATE_bm;

    cooldown:
        _delay_loop_2(cooldown_ticks);
    }
}
