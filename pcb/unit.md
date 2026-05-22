# Test unit wiring
Unit consists of 3 boards (CTRL, HV, HC; all under `pcb/`) & terminal unit.

## TU (Terminal Unit)

### Inner interface
E+, E-, P+, PG, I-SDA, I-SCL, I-GND

### Outer interface
(from left to right)

TBOUT (screw terminal block)
1. E+
2. E-

TBIN (screw terminal block)
1. +36V (P+)
2. GND (PG)

I2C (XH 3 pin socket)
1. SDA (I-SDA)
2. SCL (I-SCL)
3. GND (I-GND)

## Wiring

* TU.E+: HV.E+, HC.E+
* TU.E-: HV.E-, HC.E-
* TU.P+: HV.36V+, HC.36V+, CTRL.36V
* TU.PG: HV.36VG, HC.36VG, CTRL.GND
* (I-SDA, I-SCL, I-GND): CTRL.(SDA, SCL, GND)
