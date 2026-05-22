# Pulser Board

🚧 Under Construction 🚧

Open-source EDM board.

Explicitly designed for plunge EDM.
But probably will work for wire EDM too.

Created for https://github.com/xy-kasumi/Spark project.


## Board Status

Focused on V2.
It's currently test-phase, but it ran on real machine w/o breaking at least for 5 hours.

V2
* 10A max pulse
* Designed for reliablity and modular testability (especially stress testing)
* Testboards seem to be OK, but not yet tested in real machine.
* folders: `firmware-test-hv/`, `firmware-test-integ/`, `pcb/PULSER-testboard-*/`

* firmware: Pico2 & AVR
  * `firmware-test-hv/`: AVR (ATtiny1616)
  * `firmware-test-integ/`: Pico2 (CLI build. `./setup.sh` & `./build.sh` in the folder)
* pcb: JLCPCB friendly KiCad files

# Legacy

There was V1 (repo `PULSER-*` tags).
It was "feature-complete", but lacked reliability & testability.
