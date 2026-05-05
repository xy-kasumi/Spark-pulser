#!/usr/bin/env bash
# Idempotent bootstrap: fetch pico-sdk + arm-none-eabi toolchain into ./vendor.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
VENDOR="$HERE/vendor"
mkdir -p "$VENDOR"

# Pico SDK
SDK_TAG="2.1.0"
SDK_DIR="$VENDOR/pico-sdk"
if [ ! -d "$SDK_DIR/.git" ]; then
    echo "[setup] cloning pico-sdk $SDK_TAG (shallow, no submodules)..."
    git clone --depth 1 --branch "$SDK_TAG" \
        https://github.com/raspberrypi/pico-sdk.git "$SDK_DIR"
else
    echo "[setup] pico-sdk already present at $SDK_DIR"
fi

# ARM GNU toolchain 13.3.Rel1, Linux x86_64 — pinned by sha256.
TC_NAME="arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi"
TC_TAR="$TC_NAME.tar.xz"
TC_URL="https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/$TC_TAR"
# Set this once after the first download; subsequent runs verify.
TC_SHA256="${TC_SHA256:-95c011cee430e64dd6087c75c800f04b9c49832cc1000127a92a97f9c8d83af4}"
TC_DIR="$VENDOR/$TC_NAME"

if [ ! -d "$TC_DIR" ]; then
    if [ ! -f "$VENDOR/$TC_TAR" ]; then
        echo "[setup] downloading $TC_TAR (~150 MB)..."
        curl -fL --retry 3 -o "$VENDOR/$TC_TAR.part" "$TC_URL"
        mv "$VENDOR/$TC_TAR.part" "$VENDOR/$TC_TAR"
    fi
    if [ -n "$TC_SHA256" ]; then
        echo "[setup] verifying sha256..."
        echo "$TC_SHA256  $VENDOR/$TC_TAR" | sha256sum -c -
    else
        ACTUAL=$(sha256sum "$VENDOR/$TC_TAR" | awk '{print $1}')
        echo "[setup] WARNING: TC_SHA256 not pinned. Computed: $ACTUAL"
        echo "[setup] Edit setup.sh to set TC_SHA256=\"$ACTUAL\" for hermetic verification."
    fi
    echo "[setup] extracting toolchain (~1.5 GB) ..."
    tar -xJf "$VENDOR/$TC_TAR" -C "$VENDOR"
    rm "$VENDOR/$TC_TAR"
else
    echo "[setup] toolchain already present at $TC_DIR"
fi

echo "[setup] done. Run ./build.sh"
