# Pulser HC (High-Current) Board / Module


## Interfaces

FFC P=1mm
* 1:VCC 2:GND 3:EN 4:CURR 5:GND 6,7,8:NC

Isolated digital
* VCC: 5V or 3.3V (digital I/O voltage), <5mA
* EN: input
* CURR: input (PWM, 10kHz~200kHz)

CURR specifies output current.
* Setting delay: 10ms
* Working range: 22%(5A) - 45%(10A) (0.22A/pt)
  * Below 22%: tries to output specified current, but ripple dominates
  * Above 45%: internally clamped to 10A

Indicators
* PWR (green):
* EN (amber):


## Characteristics

### T_rise
Time between: EN rise (in input) & output current crossing 1A.

Measurement condition:
* 20V diode load
* EN L->H

Reference data
* CURR=5A, T_rise=?us
* CURR=10A, T_rise=?us


## Stress Tests

### Heat Dissipation
Measurement condition
* Fan & duct assembled
* EN=H (continuous)
* CURR=10A
* 20V diode load, short load

Visible surface temerature must not exceed 100 ℃

but... D1 120C


### Voltage Overshoot
Measurement condition
* CURR=10A
* EN=H (continuous)
* repeated open<->short, load<->open, load<->short
  * Use external MOSFET-switch to simulate load disconnect

* Output voltage must not exceed 115V at any point (threshold trigger for more than 30sec)
* Internal MOSFET (Q1, Q2) Vds & Vgs must not exceed 90% of abs. max 
