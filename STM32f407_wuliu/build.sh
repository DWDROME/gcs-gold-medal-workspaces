#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build/win-debug}"
ROOT_WIN="$(wslpath -w "$ROOT")"
BUILD_DIR_WIN="$(wslpath -w "$BUILD_DIR")"
CMAKE_EXE_WIN="${CMAKE_EXE_WIN:-C:\\Users\\ds_ev\\AppData\\Local\\stm32cube\\bundles\\cmake\\4.3.1+st.1\\bin\\cmake.exe}"
GCC_BIN_WIN="${GCC_BIN_WIN:-C:\\Users\\ds_ev\\AppData\\Local\\stm32cube\\bundles\\gnu-tools-for-stm32\\14.3.1+st.2\\bin}"

if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
  "$ROOT/cfg.sh"
elif ! grep -Fq "CMAKE_HOME_DIRECTORY:INTERNAL=$ROOT_WIN" "$BUILD_DIR/CMakeCache.txt"; then
  "$ROOT/cfg.sh"
fi

powershell.exe -NoProfile -Command "\
  \$env:PATH = '$GCC_BIN_WIN;' + \$env:PATH; \
  & '$CMAKE_EXE_WIN' --build '$BUILD_DIR_WIN' --target STM32f407_wuliu --"

echo "built: $BUILD_DIR/STM32f407_wuliu.elf"
