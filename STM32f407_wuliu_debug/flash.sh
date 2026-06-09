#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build/win-debug}"
ELF="${ELF:-$BUILD_DIR/STM32f407_wuliu.elf}"
OPENOCD_EXE="${OPENOCD_EXE:-D:\\ST\\STM32CubeIDE_2.0.0\\STM32CubeIDE\\plugins\\com.st.stm32cube.ide.mcu.externaltools.openocd.win32_2.4.300.202509300731\\tools\\bin\\openocd.exe}"
OPENOCD_SCRIPTS="${OPENOCD_SCRIPTS:-D:\\ST\\STM32CubeIDE_2.0.0\\STM32CubeIDE\\plugins\\com.st.stm32cube.ide.mcu.debug.openocd_2.3.200.202510310951\\resources\\openocd\\st_scripts}"

if [ ! -f "$ELF" ]; then
  "$ROOT/build.sh"
fi

ELF_WIN="$(wslpath -w "$ELF")"

powershell.exe -NoProfile -Command "& '$OPENOCD_EXE' -s '$OPENOCD_SCRIPTS' -f interface/cmsis-dap.cfg -c 'transport select swd' -c 'adapter speed 4000' -c 'gdb_port disabled' -c 'tcl_port disabled' -c 'telnet_port disabled' -f target/stm32f4x.cfg -c 'program {$ELF_WIN} verify reset exit'"

echo "flashed: $ELF"
