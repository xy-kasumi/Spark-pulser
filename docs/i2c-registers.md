# PULSER I2C Register Map (somewhat stale)

### Control Registers
| Address | Register      | Access      | Resets to   | Description |
|---------|---------------|-------------|-------------|-------------|
| 0x01    | POLARITY      | RW          | 0           | (legacy) 0: OFF, 1: tool positive, 2: tool negative |
| 0x02    | PULSE_CURRENT | RW          | 10 (1A)     | (legacy) Pulse current in 100mA step. 1 (100mA) ~ 200 (20A) is allowed. |
| 0x03    | TEMPERATURE   | R           | N/A         | (legacy) Current heatsink temperature in ℃. 80 means 80℃. |
| 0x04    | PULSE_DUR     | RW          | 50 (500us)  | Pulse duration in 10 us unit. 5 (50us) ~ 100 (1000us) is allowed. |
| 0x05    | MAX_DUTY      | RW          | 25 (25%)    | Maximum duty factor allowed in percent. 1~95 is allowed. |
| 0x10    | CKP_PS        | R (special) | N/A         | Create checkpoint and reads pulse stats. High 4 bit is pulse rate, low 4 bits is short rate. See "checkpointed read" for details. |
| 0x80    | TEST          | RW          | 0           | (legacy) 0: Normal operation. bit 0 (LSB) is set: Disable short detection. bit 1: Disable ignition wait. |

Register access:
* RW: read-write
* R: read-only

Invalid value writes are:
* set to safe default such as OFF (e.g. POLARITY)
* set to nearest valid value (e.g. PULSE_CURRENT)
* ignored for read-only or unused registers (e.g. TEMPERATURE)

### Checkpointed read & safety
Pulser maintains discharge statistics since last checkpoint.

Reading the CKP_PS register cause these two to happen atomically:
* returns the statistics from the last checkpoint and now, via I2C
* create new checkpoint and zeroes internal stats counters

Pulser only becomes active if and only if both are met:
* POLARITY != OFF
* CKP_PS was read within last 100ms

Thus, host must poll CKP_PS with more than 10Hz frequency.
This ensures that pulser de-energizes electrodes safely when I2C communication is disrupted by host or cable failure.

CKP_PS register consists of two values.
* higher 4 bits: R_PULSE (0~15)
* lower 4 bits: R_SHORT (0~15)

From R_PULSE and R_SHORT, host can calculate R_OPEN.

> R_OPEN = 15 - (R_PULSE + R_SHORT)
(The sum of R_PULSE & R_SHORT never exceeds 15.)

R_PULSE, R_SHORT, and R_OPEN represent time ratio between 0.0 (0/15) ~ 1.0 (15/15),
and they add up to 1.0.

At any point, pulser is one of 4 states.
* Active
  * Open: electrodes are energized but discharge was not happening
  * Short: electrodes are energized and short was happening
  * Pulse: electrodes are energized and pulse is happening
* Inactive: electrodes are not energized (POLARITY=OFF, last CKP_PS read was more than 100ms ago, cooling down to follow MAX_DUTY)

R_PULSE = (duration of "Pulse") / (duration of "Active"), etc.

Note that Inactive duration is not taken into account for the ratios. This makes the reading more consistent for different MAX_DUTY
(such as 70% for rough discharge, and 5% for finishing discharge).

### Configuration delay

Writes to POLARITY & PULSE_CURRENT immediately updates the register.
However, when register is changed during active,
* change will take effect from the next pulse
* actual EDM driver will be suppressed for 1ms (PULSE_CURRENT change) and 20ms (POLARITY change)

As an exception to the delay above, setting POLARITY to OFF immediately shutdown EDM driver.
