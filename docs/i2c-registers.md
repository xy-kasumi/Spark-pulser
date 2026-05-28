# New Register Map

* 7-bit I2C device address: 0x3c

| Address | Register | Access     |
|---------|----------|------------|
| 0x01    | CTRL     | RW         |
| 0x02    | MODE     | RW         |
| 0x03    | CURR     | RW         |
| 0x04    | TIM      | RW         |
| 0x08    | RES0     | R          |
| 0x09    | RES1     | R          |
| 0x10    | FAULT    | R + clear  |

Supported pperational range
* `CURR.curr`: 10
* `TIM.max_duty`: all range
* `TIM.dur`: 

## CTRL
Controls device run status.

`| reserved: 7 | run: 1 |`

* default: 0x00

`run := 1` activates output.
Read of `run` indicates whether device is running or not.

Device stops running when:
* `run := 0`
* `probe` mode finishes
* fault happens

When device is running, watch dog timer (WDT) of 50ms is active.
Reading `RES0` is the only way to reset WDT.
WDT timeout is considered a fault.

## MODE
**default: 0x00**

Run mode. Write fails when device is running.

`| reserved: 7 | mode: 1 |`

mode
* 0: `probe` mode
* 1: `cut` mode

### `probe` mode
Uses electrodes to detect conduction with minimum current & duration.
It does not use CURR or TIM settings.
Once it detects conduction, device stops after setting `RESULT` register.

### `cut` mode
Iso-pulse continuous run mode. Uses `CURR`, `TIM` register values.

## CURR
**default: 0x0a**

Pulse current configuration. Write fails when device is running.

`| curr: 8 |`

`curr` specifies pulse current in Amperes.
Written value is clamped to closest device-supported value.

Current pulser only supports: `curr==10`.

## TIM
**default: 0x71**

Pulse timing configuration. Write fails when device is running.

`| max_duty: 4 | dur: 4 |`

`max_duty` limits the max duty by varying minimum pulse-pulse cooldown time, as `(max_duty + 1)/16`.
* `max_duty==0`: 6%
* `max_duty==1`: 12%
* ...
* `max_duty==15`: 100%

`dur` configures good pulse duration `(dur + 1) * 50us`.
* `dur==0`: `50us`
* `dur==1`: `100us`
* ...
* `dur==15`: `800us`


## RES0..1
**default: 0x00,0x00**

Pulse result and WDT clearing.

Reading `RES0` causes WDT clearing and result value update.
Result registers shows statistics between previous read (or run start).

Host can retrieve (RES0, RES1) atomically by normal sequential read of `RES0->RES1`.

### `cut` mode
* `RES0`: `| fault: 1 | reserved: 1 | r_open: 3 | r_short: 3 |`
* `RES1`: `| num_good: 8 |`

* ratio (num_open / num_window): `r_open / 7`
* ratio (num_short / num_window): `r_short / 7`
* number of good pulse: `num_good`

The count does not include ongoing open window.

`r_open` & `r_short` can be used for EDM feedback control (open: advance, short: retract).
If there's no data to compute ratios (immediate read after start), `r_open==7` & `r_short==0` will be returned.

`num_good`, in conjunction with pulse duration, can be used to compute "effective duty",
which is a good proxy for MRR (material removal rate) and power dissipation.
* `effective duty` = `duration of good pulse / entire duration` = `pulse_dur * num_good / time_between_read`

`num_good` count saturates at 255. Host must poll fast enough to get accurate value w/o saturation.

note: since `r_{open,short}` represents ratios, `r_open + r_short <= 7` holds.

### `probe` mode
Only `RES0` is used.

* `RES0`: `| fault: 1 | reserved: 6 | detected: 1 |`
* `RES1`: `| reserved: 8 |`

Unlike in `cut` mode, reading will not clear the result.


## FAULT
**default: 0x00**

`| reserved: 6 | wdt: 1 | fault: 1 |`

`wdt==1` indicates WDT timeout has happened. Writing 1 clears this bit.
`fault==1` indicates device has some kind of fault, and cannot run. Writing 1 is no-op.


# PULSER I2C Register Map (somewhat stale)

* 7-bit I2C address: 0x3b

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
