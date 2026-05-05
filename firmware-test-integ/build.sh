#!/usr/bin/env bash
# Build wrapper: configures hermetic SDK + toolchain paths, runs cmake + make.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
VENDOR="$HERE/vendor"
TC_DIR="$VENDOR/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi"
SDK_DIR="$VENDOR/pico-sdk"

if [ ! -d "$TC_DIR/bin" ] || [ ! -d "$SDK_DIR" ]; then
    echo "[build] vendor/ missing pieces; run ./setup.sh first." >&2
    exit 1
fi

export PICO_SDK_PATH="$SDK_DIR"
export PATH="$TC_DIR/bin:$PATH"

mkdir -p "$HERE/build"
cd "$HERE/build"
cmake -DPICO_BOARD=pico2 -DCMAKE_BUILD_TYPE=Release ..
make -j

echo "[build] ELF: $HERE/build/PULSER_test_integ.elf"
