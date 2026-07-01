// SPDX-License-Identifier: BSD-2-Clause
/**
 * ATtiny1616 port of the Pico 2 controller (firmware-test-integ). Same device
 * spec: register map docs/i2c-registers.md, window model docs/operation.md.
 *
 * Single core, so the dual-core split collapses:
 *   - The pulse loop runs in the foreground (main). It is the timing-critical
 *     path and must not be disrupted by more than ~1 us in-window.
 *   - The I2C slave runs in the TWI ISR; read_reg/write_reg execute there.
 *
 * Interrupts stay enabled throughout, including during a window, because every
 * ISR path that can fire while running is a handful of cycles:
 *   - Config writes are rejected while running (the !cfg_run guard), so the slow
 *     reclamp path never runs during cutting.
 *   - RES0's ratio math would be slow, so it is precomputed: the main loop keeps
 *     a "shadow" of the RES0/RES1 bytes (recompute_res, run out-of-window), and
 *     the ISR just returns it. Reading RES0 resets the WDT and clears the window
 *     counts; stats are only ever updated between windows.
 * A TWI byte interrupt that coincides with a timing edge therefore perturbs it
 * by at most one ISR (~2 us) and never cumulatively (bytes are >=25 us apart at
 * 400 kHz, and busy-waits are referenced to the free-running TCB0, so pulse
 * width stays exact regardless of interrupts). No multi-ms clock stretching.
 *
 * Shared state needs no locks beyond brief ATOMIC_BLOCK guards on the multi-byte
 * values the ISR and pulse loop both touch (cfg_wdt_ref, stat counts).
 */
#define F_CPU 20000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/atomic.h>
#include "config.h"

static const uint8_t MODE_PROBE = 0;  // cut mode is the complementary value 1

// Permanent fault: sticky, set by HV_FAULT or the boot self-test. The device
// cannot run while set and there is no way to clear it (FAULT.fault is a no-op).
static volatile bool fault = false;

// Config + run control. Written by the TWI ISR (host) and by the pulse loop;
// read by the pulse loop each iteration. `run` is bidirectional — the host
// starts/stops it, the device clears it on probe completion or WDT timeout.
static volatile bool cfg_run = false;
static volatile bool cfg_wdt = false;             // WDT timeout latched; host clears via FAULT
static volatile bool cfg_probe_detected = false;  // probe-mode conduction result
static volatile uint16_t cfg_wdt_ref = 0;         // TCA0.CNT at last RES0 read / run start
static volatile uint8_t cfg_mode = 0;
// Register values, clamped to the active current's operating range at write time
// and kept self-consistent across the CURR->DUR->DUTY dependency chain (see
// reclamp_dur / reclamp_duty), so reads always return in-range values.
static volatile uint8_t cfg_curr = 0;  // supported current (A): 1 or 10
static volatile uint8_t cfg_dur = 0;   // DUR register byte: | reserved:1 | exp:2 | frac:5 |
static volatile uint8_t cfg_duty = 0;  // DUTY register byte: max duty = (cfg_duty + 1)/256

// Cut-mode window counts since last RES0 read: incremented by the pulse loop
// (between windows, under ATOMIC_BLOCK), zeroed by the ISR on RES0 read and on
// run-start. The WDT caps the reset interval at ~51 ms and the fastest window is
// tens of microseconds, so each count stays within uint16.
static volatile uint16_t stat_open = 0;
static volatile uint16_t stat_short = 0;
static volatile uint16_t stat_good = 0;

// Precomputed RES0/RES1 bytes ("if RES0 were read now"). The main loop refreshes
// these out-of-window (recompute_res); the ISR returns them so the RES0 handler
// stays a few cycles. res0_shadow excludes the fault bit, which the ISR ORs in
// live. See docs/i2c-registers.md.
static volatile uint8_t res0_shadow = 0;
static volatile uint8_t res1_shadow = 0;

// Supported pulse currents (A) and their operating ranges. See
// docs/i2c-registers.md "operational range".
static const uint8_t CURR_1A = 1;    // exercises HV module only
static const uint8_t CURR_10A = 10;  // HV ignites, HC carries the pulse

// 10 A duration band (us). Pulses >= DUR_10A_LONG_US get the higher duty cap.
static const uint32_t DUR_10A_MIN_US = 25;
static const uint32_t DUR_10A_MAX_US = 950;
static const uint32_t DUR_10A_LONG_US = 100;
// 1 A duration is fixed by the HV firmware (HV-only pulse).
static const uint32_t DUR_1A_US = 10;

// Max duty as a DUTY register code (duty = (code + 1)/256), floored so the
// effective duty never exceeds the rated percentage.
static const uint8_t DUTY_MIN_CODE = 2;        // ~1%
static const uint8_t DUTY_1A_MAX = 22;         // ~9%
static const uint8_t DUTY_10A_SHORT_MAX = 47;  // ~19% (25..95us band)
static const uint8_t DUTY_10A_LONG_MAX = 124;  // ~49% (100..950us band)

static const uint8_t MODE_ON_RESET = 0x00;  // probe
static const uint8_t CURR_ON_RESET = 0x0a;  // 10 A
static const uint8_t DUR_ON_RESET = 0x42;   // 100 us
static const uint8_t DUTY_ON_RESET = 0x7c;  // ~49% (10 A long-band max)

// Clamp a current request to the nearest supported value (1 A or 10 A).
static uint8_t clamp_curr(uint8_t req) {
  return req <= (CURR_1A + CURR_10A) / 2 ? CURR_1A : CURR_10A;
}

// Decode a DUR register byte to pulse duration (us). exp==3 and frac 20..31 are
// reserved; treat them as the largest in-range code so a stray write maps to the
// longest scale rather than zero.
static uint32_t dur_decode_us(uint8_t b) {
  uint8_t exp = (b >> 5) & 0x03;
  uint8_t frac = b & 0x1f;
  if (exp > 2) exp = 2;
  if (frac > 19) frac = 19;
  uint32_t mul = exp == 0 ? 10u : (exp == 1 ? 100u : 1000u);
  return mul * frac / 20u;
}

// Encode a pulse duration (us) back to a canonical DUR byte (smallest exp).
static uint8_t dur_encode(uint32_t us) {
  for (uint8_t exp = 0; exp <= 2; exp++) {
    uint32_t mul = exp == 0 ? 10u : (exp == 1 ? 100u : 1000u);
    uint32_t frac = us * 20u / mul;
    if (frac >= 1 && frac <= 19 && mul * frac / 20u == us) {
      return (uint8_t)((exp << 5) | frac);
    }
  }
  return 0;  // unreachable: every clamped duration is representable
}

// Clamp pulse duration to the operating range of the active current.
static uint32_t clamp_dur_us(uint8_t curr, uint32_t us) {
  if (curr == CURR_1A) return DUR_1A_US;  // fixed by HV firmware
  if (us < DUR_10A_MIN_US) return DUR_10A_MIN_US;
  if (us > DUR_10A_MAX_US) return DUR_10A_MAX_US;
  return us;
}

// Clamp a DUTY register code to the band allowed by the active current &
// (already-clamped) duration.
static uint8_t clamp_duty_code(uint8_t curr, uint32_t dur_us, uint8_t code) {
  uint8_t hi;
  if (curr == CURR_1A) {
    hi = DUTY_1A_MAX;
  } else {
    hi = dur_us >= DUR_10A_LONG_US ? DUTY_10A_LONG_MAX : DUTY_10A_SHORT_MAX;
  }
  if (code < DUTY_MIN_CODE) return DUTY_MIN_CODE;
  if (code > hi) return hi;
  return code;
}

// Re-clamp the stored DUTY into the band allowed by the current CURR & DUR.
static void reclamp_duty(void) {
  cfg_duty = clamp_duty_code(cfg_curr, dur_decode_us(cfg_dur), cfg_duty);
}

// Re-clamp the stored DUR into the current CURR's range (canonicalizing the
// byte), then re-clamp DUTY since its band depends on DUR.
static void reclamp_dur(void) {
  cfg_dur = dur_encode(clamp_dur_us(cfg_curr, dur_decode_us(cfg_dur)));
  reclamp_duty();
}

////////////////////////////////////////////////////////////////////////////////
// Register read/write & TWI slave (runs in the TWI ISR).

enum {
  REG_CTRL = 0x01,
  REG_MODE = 0x02,
  REG_CURR = 0x03,
  REG_DUR = 0x04,
  REG_DUTY = 0x05,
  REG_RES0 = 0x08,
  REG_RES1 = 0x09,
  REG_FAULT = 0x10,
};

// num_good captured at the last RES0 read, returned by the following RES1 read.
// Lets the host fetch (RES0, RES1) atomically via a sequential read.
static uint8_t res1_latch = 0;

static bool i2c_ptr_written = false;
static uint8_t i2c_reg_ptr = 0;

// Called from the TWI ISR (interrupts off) — no extra locking needed.
static void write_reg(uint8_t reg, uint8_t val) {
  switch (reg) {
    case REG_CTRL: {
      bool run_req = val & 0x01;
      if (!run_req) {
        cfg_run = false;
      } else if (!cfg_run && !fault) {
        // Rising edge of run: arm WDT and clear last run's results. A latched
        // wdt bit does not block restart — it is recoverable, unlike fault.
        cfg_run = true;
        cfg_wdt_ref = TCA0.SINGLE.CNT;
        cfg_probe_detected = false;
        stat_open = stat_short = stat_good = 0;
      }
      break;
    }
    case REG_MODE:
      if (!cfg_run) {  // write fails while running
        cfg_mode = val & 0x01;
      }
      break;
    case REG_CURR:
      if (!cfg_run) {
        cfg_curr = clamp_curr(val);
        reclamp_dur();  // DUR (and DUTY) band depends on CURR
      }
      break;
    case REG_DUR:
      if (!cfg_run) {
        cfg_dur = val;
        reclamp_dur();  // clamp/canonicalize DUR, then re-clamp DUTY
      }
      break;
    case REG_DUTY:
      if (!cfg_run) {
        cfg_duty = val;
        reclamp_duty();
      }
      break;
    case REG_FAULT:
      if (val & 0x02) {  // wdt bit: write 1 clears
        cfg_wdt = false;
      }
      // fault bit write is a no-op.
      break;
  }
}

// Called from the TWI ISR (interrupts off) — no extra locking needed.
static uint8_t read_reg(uint8_t reg) {
  switch (reg) {
    case REG_CTRL:
      return cfg_run ? 1 : 0;
    case REG_MODE:
      return cfg_mode;
    case REG_CURR:
      return cfg_curr;
    case REG_DUR:
      return cfg_dur;
    case REG_DUTY:
      return cfg_duty;
    case REG_RES0: {
      // Reading RES0 resets the WDT, latches RES1, and (cut mode) consumes the
      // window counts. The returned value is precomputed by recompute_res(), so
      // this stays a few cycles. RES0.fault mirrors only the permanent fault
      // (cannot run); a recoverable WDT timeout surfaces via FAULT.wdt instead.
      cfg_wdt_ref = TCA0.SINGLE.CNT;
      res1_latch = res1_shadow;
      uint8_t r = res0_shadow | (fault ? 0x80 : 0x00);
      if (cfg_mode != MODE_PROBE) {
        stat_open = stat_short = stat_good = 0;  // probe result is not cleared by reading
      }
      return r;
    }
    case REG_RES1:
      return res1_latch;
    case REG_FAULT:
      return (cfg_wdt ? 0x02 : 0x00) | (fault ? 0x01 : 0x00);
  }
  return 0;
}

// TWI slave: register-pointer protocol. First byte of a write is the register
// pointer; subsequent writes auto-increment. Reads return successive registers,
// auto-incrementing (so the host can read RES0,RES1 in one sequential read).
ISR(TWI0_TWIS_vect) {
  uint8_t s = TWI0.SSTATUS;

  if (s & (TWI_BUSERR_bm | TWI_COLL_bm)) {
    // Bus error / collision: drop the transaction and re-arm.
    i2c_ptr_written = false;
    TWI0.SSTATUS = TWI_BUSERR_bm | TWI_COLL_bm;
    TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
    return;
  }

  if (s & TWI_APIF_bm) {
    if (s & TWI_AP_bm) {
      // Address match: ACK and proceed. Pointer state persists across a
      // repeated start; it is reset only on stop.
      TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
    } else {
      // Stop condition.
      i2c_ptr_written = false;
      TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
    }
    return;
  }

  if (s & TWI_DIF_bm) {
    if (s & TWI_DIR_bm) {
      // Master read: we transmit. After the first byte, RXACK reflects the
      // master's response to the previous byte; NACK means it wants no more.
      if (s & TWI_RXACK_bm) {
        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
      } else {
        TWI0.SDATA = read_reg(i2c_reg_ptr++);
        TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
      }
    } else {
      // Master write: we receive.
      uint8_t d = TWI0.SDATA;
      if (!i2c_ptr_written) {
        i2c_reg_ptr = d;
        i2c_ptr_written = true;
      } else {
        write_reg(i2c_reg_ptr++, d);
      }
      TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
    }
    return;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Init.

static void clock_init(void) {
  // Run at full 20 MHz: disable prescaler (default is /6).
  CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0;
}

static void pins_init(void) {
  // Outputs low; HV_FAULT input with pull-up (active-low). I2C pins are driven
  // by TWI0 when enabled (board provides external pull-ups).
  VPORTA.OUT &= ~(HV_EN_bm | HC_EN_bm | HC_CURR_bm);
  VPORTA.DIR |= HV_EN_bm | HC_EN_bm | HC_CURR_bm;
  VPORTA.DIR &= ~(HV_CURR_bm | HV_FAULT_bm);
  PORTA.PIN3CTRL = PORT_PULLUPEN_bm;  // HV_FAULT (PA3)
}

static void timers_init(void) {
  // TCB0: free-running 16-bit at CLK_PER (50 ns/tick) for intra-window timing.
  TCB0.CCMP = 0xFFFF;
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;

  // TCA0: free-running 16-bit at CLK_PER/1024 (51.2 us/tick) — software WDT clock.
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1024_gc | TCA_SINGLE_ENABLE_bm;
}

static void twi_init(void) {
  TWI0.SADDR = I2C_DEV_ADDR << 1;
  TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm | TWI_ENABLE_bm;
}

////////////////////////////////////////////////////////////////////////////////
// Pulse loop (foreground).

typedef enum { WIN_OPEN, WIN_SHORT, WIN_GOOD } window_type_t;

static inline bool hv_curr(void) { return VPORTA.IN & HV_CURR_bm; }
static inline void hv_en(bool on) {
  if (on)
    VPORTA.OUT |= HV_EN_bm;
  else
    VPORTA.OUT &= ~HV_EN_bm;
}
static inline void hc_en(bool on) {
  if (on)
    VPORTA.OUT |= HC_EN_bm;
  else
    VPORTA.OUT &= ~HC_EN_bm;
}

// Busy-wait via the free-running TCB0. Safe with interrupts on or off and for
// long waits: each chunk's tick count stays within uint16 (2000 us * 20 < 2^16).
static void wait_us(uint32_t us) {
  while (us > 0) {
    uint16_t chunk = us > 2000u ? 2000u : (uint16_t)us;
    uint16_t start = TCB0.CNT;
    uint16_t target = chunk * TICKS_PER_US;
    while ((uint16_t)(TCB0.CNT - start) < target) {
    }
    us -= chunk;
  }
}

// One cut-mode window. HV.EN==L && HC.EN==L on entry and exit. `use_hc` selects
// the good-pulse path: 10 A hands HV over to HC; 1 A pulses HV alone.
//
// Runs with interrupts enabled. Pulse width stays exact because the busy-waits
// are referenced to the free-running TCB0; a coincident TWI ISR perturbs an edge
// by at most its own (~2 us) duration.
static window_type_t cut_window(uint32_t pulse_dur_us, uint8_t duty_byte,
                                bool use_hc) {
  hv_en(true);

  // Wait ignition.
  uint16_t t0 = TCB0.CNT;
  window_type_t wt;
  while (true) {
    uint16_t dt = (uint16_t)(TCB0.CNT - t0);
    bool cond = hv_curr();
    if (dt <= T_TRAN_TICKS) {
      // ignore transient period data
    } else if (dt <= T_IG_MAX_TICKS) {
      if (cond) {
        wt = dt <= T_IG_SHORT_TICKS ? WIN_SHORT : WIN_GOOD;
        break;
      }
    } else {
      wt = WIN_OPEN;
      break;
    }
  }

  if (wt == WIN_OPEN) {
    hv_en(false);
    wait_us(CD_OPEN_US);
    return WIN_OPEN;
  } else if (wt == WIN_SHORT) {
    hv_en(false);
    wait_us(CD_SHORT_US);
    return WIN_SHORT;
  }

  // Good pulse.
  if (use_hc) {
    // 10 A: keep HV.EN for PULSE_HANDOVER_US while HC warms up, then hand over
    // to HC for the rest of the pulse. Actual HV pulse shape is set by HV
    // firmware.
    hc_en(true);
    wait_us(PULSE_HANDOVER_US);
    hv_en(false);
    wait_us(pulse_dur_us - PULSE_HANDOVER_US);
    hc_en(false);
  } else {
    // 1 A: HV-only pulse; HC stays off. The HV firmware fixes the pulse width
    // (~10 us), so we just hold HV.EN for the configured duration.
    wait_us(pulse_dur_us);
    hv_en(false);
  }

  // cool_duty enforces the max-duty config. max_duty = (duty_byte + 1)/256, so
  // cool = pulse * (256/(duty+1) - 1) = pulse * (255 - duty) / (duty + 1).
  uint32_t cool_duty = pulse_dur_us * (uint32_t)(255 - duty_byte) / (duty_byte + 1);
  wait_us(cool_duty > CD_GOOD_US ? cool_duty : CD_GOOD_US);
  return WIN_GOOD;
}

// Returns true if conducted. HV.EN is OFF afterwards in all cases.
// Probe cycle: 500us (100us check phase + 400us wait, to reduce electrolysis).
// Runs with interrupts enabled (see cut_window).
static bool probe_window(void) {
  uint16_t t0 = TCB0.CNT;
  hv_en(true);

  bool result = false;
  while (true) {
    uint16_t dt = (uint16_t)(TCB0.CNT - t0);
    bool cond = hv_curr();

    if (dt <= T_TRAN_TICKS) {
      // ignore transient response
    } else if (dt <= PROBE_CHECK_TICKS) {
      // latch to true. Immediately turn off to minimize electrode damage.
      if (cond) {
        result = true;
        hv_en(false);
      }
    } else if (dt < PROBE_CYCLE_TICKS) {
      hv_en(false);
    } else {
      break;
    }
  }
  return result;
}

// Permanent fault: outputs safe, sticky flag set, run cleared. Interrupts stay
// enabled so the host can still read the FAULT register.
static void enter_fault(void) {
  hv_en(false);
  hc_en(false);
  fault = true;
  cfg_run = false;
}

// Refresh the precomputed RES0/RES1 shadow bytes. Run out-of-window (the divisions
// are the slow part the RES0 ISR avoids). The shadow lags the live counts by at
// most the window that just finished, which is negligible against the ratios.
static void recompute_res(void) {
  if (cfg_mode == MODE_PROBE) {
    res1_shadow = 0;
    res0_shadow = cfg_probe_detected ? 0x01 : 0x00;
    return;
  }
  uint16_t n_open, n_short, n_good;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    n_open = stat_open;
    n_short = stat_short;
    n_good = stat_good;
  }
  res1_shadow = n_good > 255 ? 255 : (uint8_t)n_good;

  uint16_t n_window = n_open + n_short + n_good;
  uint8_t r_open, r_short;
  if (n_window == 0) {
    r_open = 7;  // no data yet: report fully open
    r_short = 0;
  } else {
    // Floor division keeps r_open + r_short <= 7 (since n_good >= 0).
    r_open = (uint8_t)((uint32_t)n_open * 7 / n_window);
    r_short = (uint8_t)((uint32_t)n_short * 7 / n_window);
  }
  res0_shadow = (uint8_t)((r_open << 3) | r_short);
}

int main(void) {
  clock_init();
  pins_init();
  timers_init();

  // Seed registers to reset defaults, keeping the DUR/DUTY invariant in range.
  cfg_mode = MODE_ON_RESET;
  cfg_curr = clamp_curr(CURR_ON_RESET);
  cfg_dur = DUR_ON_RESET;
  cfg_duty = DUTY_ON_RESET;
  reclamp_dur();
  recompute_res();  // seed the shadow before the TWI ISR can serve a RES0 read

  twi_init();
  sei();

  // Boot self-test: HV_CURR is expected L (testboard pulls it down). H means a
  // hardware fault. (firmware-test-integ blinked an LED here; this board has no
  // MCU LED, so latch the permanent fault instead.)
  wait_us(1000);  // pull-up + comparator settle
  if (hv_curr()) {
    enter_fault();
  }

  while (true) {
    // Refresh the RES0/RES1 shadow from the previous iteration's stats, so the
    // ISR's RES0 read stays cheap. Covers cut/probe/idle and the post-run-start
    // reset uniformly.
    recompute_res();

    // Atomic snapshot of config + WDT reference (multi-byte values shared with
    // the ISR). Also sample the hardware fault input.
    bool run, fault_now;
    uint8_t mode, curr, dur_byte, duty_byte;
    uint16_t wdt_ref;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      run = cfg_run;
      mode = cfg_mode;
      curr = cfg_curr;
      dur_byte = cfg_dur;
      duty_byte = cfg_duty;
      wdt_ref = cfg_wdt_ref;
    }
    fault_now = fault || !(VPORTA.IN & HV_FAULT_bm);  // HV_FAULT is active-low

    if (fault_now) {
      enter_fault();
      wait_us(100);  // idle; interrupts on, ISR serves FAULT reads
      continue;
    }

    if (!run) {
      wait_us(100);
      continue;
    }

    // WDT: host must read RES0 within WDT_TIMEOUT_US. Timeout latches wdt and
    // stops the device (recoverable).
    if ((uint16_t)(TCA0.SINGLE.CNT - wdt_ref) >= WDT_TIMEOUT_TICKS) {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cfg_wdt = true;
        cfg_run = false;
      }
      continue;
    }

    if (mode == MODE_PROBE) {
      if (probe_window()) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          cfg_probe_detected = true;
          cfg_run = false;  // probe stops on detection
        }
      }
    } else {
      uint32_t pulse_dur_us = dur_decode_us(dur_byte);
      window_type_t w = cut_window(pulse_dur_us, duty_byte, curr == CURR_10A);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
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
      }
    }
  }
}
