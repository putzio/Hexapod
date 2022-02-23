# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles" Generator, CMake Version 3.22

# compile ASM with C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-gcc.exe
# compile C with C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-gcc.exe
# compile CXX with C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-g++.exe
ASM_DEFINES = -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_UART=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CORE=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UTIL=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_TARGET_NAME=\"orchestra\" -DPICO_USE_BLOCKED_RAM=0

ASM_INCLUDES = -IC:\PICO\pico-sdk\src\common\pico_stdlib\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_gpio\include -IC:\PICO\pico-sdk\src\common\pico_base\include -IC:\PICO\orchestra\build\generated\pico_base -IC:\PICO\pico-sdk\src\boards\include -IC:\PICO\pico-sdk\src\rp2_common\pico_platform\include -IC:\PICO\pico-sdk\src\rp2040\hardware_regs\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_base\include -IC:\PICO\pico-sdk\src\rp2040\hardware_structs\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_claim\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_sync\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_uart\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_divider\include -IC:\PICO\pico-sdk\src\common\pico_time\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_timer\include -IC:\PICO\pico-sdk\src\common\pico_sync\include -IC:\PICO\pico-sdk\src\common\pico_util\include -IC:\PICO\pico-sdk\src\rp2_common\pico_runtime\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_clocks\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_irq\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_resets\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_pll\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_vreg\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_watchdog\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_xosc\include -IC:\PICO\pico-sdk\src\rp2_common\pico_printf\include -IC:\PICO\pico-sdk\src\rp2_common\pico_bootrom\include -IC:\PICO\pico-sdk\src\common\pico_bit_ops\include -IC:\PICO\pico-sdk\src\common\pico_divider\include -IC:\PICO\pico-sdk\src\rp2_common\pico_double\include -IC:\PICO\pico-sdk\src\rp2_common\pico_int64_ops\include -IC:\PICO\pico-sdk\src\rp2_common\pico_float\include -IC:\PICO\pico-sdk\src\rp2_common\pico_malloc\include -IC:\PICO\pico-sdk\src\rp2_common\boot_stage2\include -IC:\PICO\pico-sdk\src\common\pico_binary_info\include -IC:\PICO\pico-sdk\src\rp2_common\pico_stdio\include -IC:\PICO\pico-sdk\src\rp2_common\pico_stdio_uart\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_pwm\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_adc\include

ASM_FLAGS = -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG -ffunction-sections -fdata-sections

C_DEFINES = -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_UART=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CORE=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UTIL=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_TARGET_NAME=\"orchestra\" -DPICO_USE_BLOCKED_RAM=0

C_INCLUDES = -IC:\PICO\pico-sdk\src\common\pico_stdlib\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_gpio\include -IC:\PICO\pico-sdk\src\common\pico_base\include -IC:\PICO\orchestra\build\generated\pico_base -IC:\PICO\pico-sdk\src\boards\include -IC:\PICO\pico-sdk\src\rp2_common\pico_platform\include -IC:\PICO\pico-sdk\src\rp2040\hardware_regs\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_base\include -IC:\PICO\pico-sdk\src\rp2040\hardware_structs\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_claim\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_sync\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_uart\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_divider\include -IC:\PICO\pico-sdk\src\common\pico_time\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_timer\include -IC:\PICO\pico-sdk\src\common\pico_sync\include -IC:\PICO\pico-sdk\src\common\pico_util\include -IC:\PICO\pico-sdk\src\rp2_common\pico_runtime\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_clocks\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_irq\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_resets\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_pll\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_vreg\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_watchdog\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_xosc\include -IC:\PICO\pico-sdk\src\rp2_common\pico_printf\include -IC:\PICO\pico-sdk\src\rp2_common\pico_bootrom\include -IC:\PICO\pico-sdk\src\common\pico_bit_ops\include -IC:\PICO\pico-sdk\src\common\pico_divider\include -IC:\PICO\pico-sdk\src\rp2_common\pico_double\include -IC:\PICO\pico-sdk\src\rp2_common\pico_int64_ops\include -IC:\PICO\pico-sdk\src\rp2_common\pico_float\include -IC:\PICO\pico-sdk\src\rp2_common\pico_malloc\include -IC:\PICO\pico-sdk\src\rp2_common\boot_stage2\include -IC:\PICO\pico-sdk\src\common\pico_binary_info\include -IC:\PICO\pico-sdk\src\rp2_common\pico_stdio\include -IC:\PICO\pico-sdk\src\rp2_common\pico_stdio_uart\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_pwm\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_adc\include

C_FLAGS = -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG -ffunction-sections -fdata-sections

CXX_DEFINES = -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_UART=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CORE=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UTIL=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_TARGET_NAME=\"orchestra\" -DPICO_USE_BLOCKED_RAM=0

CXX_INCLUDES = -IC:\PICO\pico-sdk\src\common\pico_stdlib\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_gpio\include -IC:\PICO\pico-sdk\src\common\pico_base\include -IC:\PICO\orchestra\build\generated\pico_base -IC:\PICO\pico-sdk\src\boards\include -IC:\PICO\pico-sdk\src\rp2_common\pico_platform\include -IC:\PICO\pico-sdk\src\rp2040\hardware_regs\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_base\include -IC:\PICO\pico-sdk\src\rp2040\hardware_structs\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_claim\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_sync\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_uart\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_divider\include -IC:\PICO\pico-sdk\src\common\pico_time\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_timer\include -IC:\PICO\pico-sdk\src\common\pico_sync\include -IC:\PICO\pico-sdk\src\common\pico_util\include -IC:\PICO\pico-sdk\src\rp2_common\pico_runtime\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_clocks\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_irq\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_resets\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_pll\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_vreg\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_watchdog\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_xosc\include -IC:\PICO\pico-sdk\src\rp2_common\pico_printf\include -IC:\PICO\pico-sdk\src\rp2_common\pico_bootrom\include -IC:\PICO\pico-sdk\src\common\pico_bit_ops\include -IC:\PICO\pico-sdk\src\common\pico_divider\include -IC:\PICO\pico-sdk\src\rp2_common\pico_double\include -IC:\PICO\pico-sdk\src\rp2_common\pico_int64_ops\include -IC:\PICO\pico-sdk\src\rp2_common\pico_float\include -IC:\PICO\pico-sdk\src\rp2_common\pico_malloc\include -IC:\PICO\pico-sdk\src\rp2_common\boot_stage2\include -IC:\PICO\pico-sdk\src\common\pico_binary_info\include -IC:\PICO\pico-sdk\src\rp2_common\pico_stdio\include -IC:\PICO\pico-sdk\src\rp2_common\pico_stdio_uart\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_pwm\include -IC:\PICO\pico-sdk\src\rp2_common\hardware_adc\include

CXX_FLAGS = -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG -ffunction-sections -fdata-sections -fno-exceptions -fno-unwind-tables -fno-rtti -fno-use-cxa-atexit

