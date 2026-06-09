#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build/win-debug}"
ROOT_WIN="$(wslpath -w "$ROOT")"
BUILD_DIR_WIN="$(wslpath -w "$BUILD_DIR")"
TOOLCHAIN_FILE_WIN="$(wslpath -w "$ROOT/cmake/gcc-arm-none-eabi.cmake")"
CMAKE_EXE_WIN="${CMAKE_EXE_WIN:-C:\\Users\\ds_ev\\AppData\\Local\\stm32cube\\bundles\\cmake\\4.3.1+st.1\\bin\\cmake.exe}"
GCC_BIN_WIN="${GCC_BIN_WIN:-C:\\Users\\ds_ev\\AppData\\Local\\stm32cube\\bundles\\gnu-tools-for-stm32\\14.3.1+st.2\\bin}"
GCC_EXE_WIN="${GCC_EXE_WIN:-$GCC_BIN_WIN\\arm-none-eabi-gcc.exe}"
GXX_EXE_WIN="${GXX_EXE_WIN:-$GCC_BIN_WIN\\arm-none-eabi-g++.exe}"

mkdir -p "$BUILD_DIR"
rm -f "$BUILD_DIR/CMakeCache.txt"

powershell.exe -NoProfile -Command "\
  \$env:PATH = '$GCC_BIN_WIN;' + \$env:PATH; \
  & '$CMAKE_EXE_WIN' \
    -S '$ROOT_WIN' \
    -B '$BUILD_DIR_WIN' \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_TOOLCHAIN_FILE='$TOOLCHAIN_FILE_WIN' \
    -DCMAKE_C_COMPILER='$GCC_EXE_WIN' \
    -DCMAKE_CXX_COMPILER='$GXX_EXE_WIN' \
    -DCMAKE_ASM_COMPILER='$GCC_EXE_WIN'"

echo "configured: $BUILD_DIR"
