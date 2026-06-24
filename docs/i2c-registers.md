# Register Map

* 7-bit I2C device address: 0x3c

| Address | Register | Access     |
|---------|----------|------------|
| 0x01    | CTRL     | RW         |
| 0x02    | MODE     | RW         |
| 0x03    | CURR     | RW         |
| 0x04    | DUR      | RW         |
| 0x05    | DUTY     | RW         |
| 0x08    | RES0     | R          |
| 0x09    | RES1     | R          |
| 0x10    | FAULT    | R + clear  |

Supported operational range
* current: 10A
  * duration: 100us~950us / duty: 1%~49%
  * duration: 25us~95us / duty: 1%~19%
* current: 1A
  * duration: 10us / duty: 1%~9%

Auto-clamping works like:
* CURR: calmps on its own
* DUR: clamps based on current CURR
* DUTY: clamps based on current CURR & DUR

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

## DUR
**default: 0x42** (100us)

`| reserved: 1 | exp: 2 | frac: 5 |`

Allowed `frac` is 0..19 (20..31 are reserved).

* exp==0: 10us
* exp==1: 100us
* exp==2: 1000us
* exp==3: reserved

`pulse duration = multiplier * (frac / 20)`

e.g. 0x01 (exp==0, frac==1): 10us x (1 / 20) = 0.5us (min positive value)
e.g. 0x42 (exp==2, frac==2): 1000us x (2 / 20) = 100us
e.g. 0x53 (exp==2, frac==19): 1000us x (19 / 20) = 950us (max value)


## DUTY
**default: 0x7c** (49%)

Limits the max duty by varying minimum cooldown time between pulses, as `(DUTY + 1) / 256`.


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
