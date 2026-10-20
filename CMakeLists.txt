cmake_minimum_required(VERSION 3.20)

project(PII-stm32-fault-tolerance-system C ASM)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain ARM GCC
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

# Flag Cortex-M3 (STM32L152RE)
set(CMAKE_C_FLAGS "-mcpu=cortex-m3 -mthumb -Wall -Wextra -fdata-sections -ffunction-sections -O0 -g3" CACHE INTERNAL "")
set(CMAKE_ASM_FLAGS "-mcpu=cortex-m3 -mthumb -x assembler-with-cpp" CACHE INTERNAL "")

# Definizioni del modello e driver HAL
add_definitions(
    -DSTM32L152xE
    -DUSE_HAL_DRIVER
)

# Header include
include_directories(
    Core/Inc
    Drivers/STM32L1xx_HAL_Driver/Inc
    Drivers/STM32L1xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32L1xx/Include
    Drivers/CMSIS/Include
)

# Raccoglie i sorgenti C e assembly
file(GLOB_RECURSE SOURCES
    "Core/Src/*.c"
    "Core/Startup/startup_stm32l152retx.s"
    "Drivers/STM32L1xx_HAL_Driver/Src/*.c"
)

# Esclusione template non necessari del driver HAL
list(FILTER SOURCES EXCLUDE REGEX ".*_template\\.c$")

add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Script del Linker
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32L152RETX_FLASH.ld")

target_link_options(${PROJECT_NAME}.elf PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -T${LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map
    --specs=nano.specs
    --specs=nosys.specs
    -u _printf_float
    -u _printf_float
    -u _printf_float
)

# Generazione binario .bin, .hex e report memoria
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} --format=berkeley ${PROJECT_NAME}.elf
)
