# Pulser Board

🚧 Under Construction 🚧

Open-source EDM board.

Explicitly designed for plunge EDM.
But probably will work for wire EDM too.

Created for https://github.com/xy-kasumi/Spark project.

## Board Status

There are V1 & V2.

V1
* 20A max pulse
* What Spark host expects. Implements registers. Worked in real setup.
  * However, known to break in duration of several minutes ~ tens of hours during actual use.
    * N=2/2 breaks
  * Undebuggable monolith with little future
* folders: `firmware/`, `pcb/PULSER/`

V2
* 10A max pulse
* Designed for reliablity and modular testability (especially stress testing)
* Testboards seem to be OK, but not yet tested in real machine.
* folders: `firmware-test-hv/`, `firmware-test-integ/`, `pcb/PULSER-testboard-*/`

## Building

* docs: manuals for using or building the board
* firmware: Pico2 & AVR
  * `firmware/`: Pico2 based firmware (Open `firmware/` folder in VSCode, and use Raspberry Pi extension to build & flash.)
  * `firmware-test-hv/`: AVR (ATtiny1616)
  * `firmware-test-integ/`: Pico2 (CLI build. `./setup.sh` & `./build.sh` in the folder)
* pcb: JLCPCB friendly KiCad files

These are necessary for building, testing, and using the machine.
