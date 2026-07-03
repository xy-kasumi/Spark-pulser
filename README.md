# Pulser Board

🚧 Under Construction 🚧

Open-source EDM board.

Explicitly designed for plunge EDM.
But probably will work for wire EDM too.

Created for https://github.com/xy-kasumi/Spark project.


## Board Status

V2
* 10A max pulse
* Designed for reliablity and modular testability (especially stress testing)

* firmware: Pico2 & AVR
  * `firmware-test-hv/`: AVR (ATtiny1616)
  * `firmware-test-integ/`: Pico2 (CLI build. `./setup.sh` & `./build.sh` in the folder)
* pcb: JLCPCB friendly KiCad files

testboard (30hr+ successful op in real settings)
* ctrl: r1 (breadboard, pico2), w/ `firmware-test-integ/`
* hv: r1-r3 (ATtiny1616), w/ `firmware-test-hv/`
* hc: r1-r2

prod (not tested yet)
* ctrl: r2
* hv: r4
* hc: r3

# Legacy

There was V1 (repo `PULSER-*` tags).
It was "feature-complete", but lacked reliability & testability.
