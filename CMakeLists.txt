cmake_minimum_required(VERSION 3.14)

# Specify the cross compiler before project()
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# ARM GCC Toolchain
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

# Skip compiler checks
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

project(ahrs_stm32 VERSION 1.0.0 LANGUAGES C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MCU settings for STM32F446RE
set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(MCU_DEFINES "-DSTM32F446xx")

# Compiler flags
set(CMAKE_C_FLAGS "${MCU_FLAGS} ${MCU_DEFINES} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_CXX_FLAGS "${MCU_FLAGS} ${MCU_DEFINES} -Wall -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS "${MCU_FLAGS} -x assembler-with-cpp")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -T${CMAKE_SOURCE_DIR}/STM32F446XX_FLASH.ld -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map --specs=nano.specs --specs=nosys.specs")

# Build type
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/drivers/cmsis/include
  ${PROJECT_SOURCE_DIR}/drivers/cmsis/device/st/stm32f4xx/include
)

# Source files
set(SOURCES
  # System
  startup_stm32f446xx.s
  src/system_stm32f4xx.c

  # MCAL
  src/mcal/cmsis_i2c.cpp
  src/mcal/cmsis_uart.cpp
  src/mcal/clk_config.cpp

  # Drivers
  src/drivers/refactor_me_mpu9250.cpp

  # Filters
  src/filters/madgwick.cpp
  src/filters/ekf.cpp
  src/math/quaternion.cpp
  src/math/dcm.cpp

  # Main
  main.cpp
)

# Create executable
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}.elf Eigen3::Eigen)

# Enable float support for printf/snprintf
target_link_options(${PROJECT_NAME}.elf PRIVATE "-Wl,-u,_printf_float")

# Generate .hex and .bin files
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
  COMMENT "Building ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
)

# Print size
add_custom_target(size ALL
  COMMAND ${CMAKE_SIZE} --format=berkeley ${PROJECT_NAME}.elf
  DEPENDS ${PROJECT_NAME}.elf
  COMMENT "Calculating size:"
)
