# Pulser HV (High-Voltage) Board / Module

Features

* 1A (approx), 95V output
* Max 10us pulse


## Interfaces

### Mechanical
Terminals
* I+/I-: 36V DC, max draw 2A
* O+/O-: pulse output

FFC P=1mm
* 1:VCC 2:GND 3:EN 4:CURR 5:GND 6:!FAULT 7,8:NC

### Electric
I-/O- shares power ground. All pins in FFC connector is isolated from I/O power rails.

Isolated digital
* VCC: 5V or 3.3V (digital I/O voltage), <5mA
* EN: input
* CURR: output
* !FAULT: output

Indicators
* PWR/STAT (green): uC is running
* CURR (amber): output current is detected
* ERR/FLT (red): uC has detected hardware/firmware fault (latches until power cycle)

When EN is H, voltage is applied to output.
CURR indicates current is flowing through output.

Output is disabled, and kept disabled when
* 10us passed since CURR rise
* EN is lowered

To start next cycle, EN must be lowered and risen again.

For thermal protection, HV board limits duty to 10%.
When CURR rises, internal cooldown counting of 100us starts.
EN rise is ignored until 100us passes.
Host is recommended to wait 110us or more before raising EN again after CURR detection.


## Characteristics

### T_detect
Time between output current & CURR rise.

Measurement condition
* 20V diode load
* Measure EN rise - CURR rise (provides upper bound, b/c current output rise time is also included)

Reference data
* xxx us


## Stress Test

### Heat Dissipation
Measurement condition
* fan & duct assembled
* short load
* EN: max density pulse (on:10us, off:90us)

Visible surface temperature must not exceed 100℃

### Voltage Overshoot
Measurement condition
* CURR=10A
* EN: pulse (10us)
* repeated open<->short, load<->open, load<->short
  * Use external MOSFET-switch to simulate load disconnect

* Output voltage must not exceed 115V at any point (threshold trigger for more than 30sec)
* Internal MOSFET (Q1, Q2) Vds & Vgs must not exceed 90% of abs. max 
