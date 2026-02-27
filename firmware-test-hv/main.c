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

/* ── Pin initialization ───────────────────────────────────────────── */

static void pins_init() {
    // STAT LED PC3: output, initially off
    VPORTC.DIR |= LED_STAT_bm;
    VPORTC.OUT &= ~LED_STAT_bm;

    // DIP switches PB0–PB5: input with pull-ups (open = high, closed = GND)
    VPORTB.DIR &= ~(PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm);
    PORTB.PIN0CTRL = PORT_PULLUPEN_bm;
    PORTB.PIN1CTRL = PORT_PULLUPEN_bm;
    PORTB.PIN2CTRL = PORT_PULLUPEN_bm;
    PORTB.PIN3CTRL = PORT_PULLUPEN_bm;
    PORTB.PIN4CTRL = PORT_PULLUPEN_bm;
    PORTB.PIN5CTRL = PORT_PULLUPEN_bm;

    // GATE PA4: output, initially low
    VPORTA.DIR |= GATE_bm;
    VPORTA.OUT &= ~GATE_bm;

    // EN PC0: input, internal pull-up (default high)
    VPORTC.DIR &= ~EN_bm;
    PORTC.PIN0CTRL = PORT_PULLUPEN_bm;

    // CURR PC1: input, no pull (driven by external comparator)
    VPORTC.DIR &= ~CURR_bm;
}

/* ── Config types & lookup tables ─────────────────────────────────── */

typedef struct {
    uint16_t pulse_dur_us;
    uint16_t cooldown_dur_us;
    uint16_t thresh_ma;
} config_t;

static const uint16_t LUT_PULSE_US[4]    = {  1,    2,    5,   10 };
static const uint16_t LUT_COOLDOWN_US[4]  = { 100, 1000,  100,  100 };
static const uint16_t LUT_THRESH_MA[4]    = {  50,  100,  200,  500 };

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

static void dac_init(uint16_t thresh_ma) {
    uint8_t dac_val = (uint16_t)thresh_ma * 128 / 434;

    VREF.CTRLA = VREF_DAC0REFSEL_4V34_gc;
    DAC0.DATA  = dac_val;
    DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm;
}

/* ── Config read ──────────────────────────────────────────────────── */

static bool dip(uint8_t pin_bm) {
    return !(VPORTB.IN & pin_bm);   // ON = GND = reads 0 → true
}

static config_t config_read() {
    // Each pair: lower pin is MSB, upper pin is LSB
    uint8_t pulse_idx    = (dip(PIN0_bm) << 1) | dip(PIN1_bm);
    uint8_t cooldown_idx = (dip(PIN2_bm) << 1) | dip(PIN3_bm);
    uint8_t thresh_idx   = (dip(PIN4_bm) << 1) | dip(PIN5_bm);

    return (config_t){
        .pulse_dur_us    = LUT_PULSE_US   [pulse_idx],
        .cooldown_dur_us = LUT_COOLDOWN_US[cooldown_idx],
        .thresh_ma       = LUT_THRESH_MA  [thresh_idx],
    };
}

/* ── Main ─────────────────────────────────────────────────────────── */

int main() {
    // Run at full 20 MHz: disable prescaler (default is /6)
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0;

    pins_init();
    config_t cfg = config_read();
    dac_init(cfg.thresh_ma);
    stat_led(true);

    // At 20 MHz, _delay_loop_2 = 4 cycles/iter = 0.2 µs → ticks = µs × 5
    uint16_t pulse_ticks    = cfg.pulse_dur_us * 5;
    uint16_t cooldown_ticks = cfg.cooldown_dur_us * 5;

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
