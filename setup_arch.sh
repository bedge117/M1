#!/bin/bash
set -e

echo "--- Preparing Arch Linux Build Environment ---"

DEPENDS=(cmake ninja git arm-none-eabi-gcc arm-none-eabi-newlib)

sudo pacman -S --needed --noconfirm "${DEPENDS[@]}"

echo "--- Dependency Setup complete. Rerun make  ---"