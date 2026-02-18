#!/bin/bash
set -e

echo "Updating submodules..."
git submodule update --init --recursive

BUILD_DIR="build"
OUTPUT_DIR="artifacts"
mkdir -p $BUILD_DIR $OUTPUT_DIR

echo "Configuring CMake..."
cmake -G Ninja -B $BUILD_DIR \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "Compiling..."
cmake --build $BUILD_DIR --parallel $(nproc)

echo "Collecting artifacts to ./$OUTPUT_DIR"
cp $BUILD_DIR/*.bin $BUILD_DIR/*.elf $BUILD_DIR/*.hex $OUTPUT_DIR/ 2>/dev/null || true

echo "Success! Firmware is in $(pwd)/$OUTPUT_DIR"