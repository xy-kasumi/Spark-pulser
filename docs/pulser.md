# Pulser V2

Features
* Iso-pulse control
* Max 10A pulse current
* Isolated I2C control & feedback interface

See [operation model](./operation.md) for how output works.

## Interface

Power
Terminal Block
* E+, E-, 36V+, 36VG

note: E- and 36VG is internally connected

Comm
XH 4 pin, I2C Fast Mode Plus
* 1:VCC 2:SDA 3:SCL 4:GND

Comm pins are isolated from power rails.
VCC must be 3.3V - 5V, providing I2C voltage reference.

Indicators
* OK (green): indicates succesful boot up of the module
* RUN (amber): indicates output is enabled
  * Warning: OFF does NOT means output is safe to touch (might still be charged due to residual capacity)
* ERR (red): indicates fault

See [I2C Register Map](./i2c-registers.md) for control.

## Mounting

TBD...
