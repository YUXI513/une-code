cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME DEMO)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Share the board crystal value with both application and STM32_Drivers sources.
# This also overrides the CMSIS system file's generic 24 MHz HSE default.
target_compile_definitions(stm32cubemx INTERFACE HSE_VALUE=8000000U)

# Safe default: ADC monitoring only, never start the Buck power PWM.
# Restore the original open-loop application only by explicitly setting OFF.
option(ADC_ONLY_TEST "ADC/OLED test with PA10 and PB13 held low" ON)
target_sources(STM32_Drivers PRIVATE
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/oled.c
    Core/Src/buck_pwm.c
    Core/Src/buck_adc.c
    Core/Src/buck_pid.c
    Core/Src/buck_control.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ADC_ONLY_TEST=$<BOOL:${ADC_ONLY_TEST}>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Separate diagnostic target: with ADC_ONLY_TEST=OFF, only PA10 switches.
# With the default ADC_ONLY_TEST=ON, this target also keeps both pins low.
# Build explicitly with: cmake --build --preset Debug --target DEMO_PA10_TEST
# EXCLUDE_FROM_ALL leaves this target out of the default build.
get_target_property(DEMO_Application_Sources ${CMAKE_PROJECT_NAME} SOURCES)
add_executable(DEMO_PA10_TEST EXCLUDE_FROM_ALL ${DEMO_Application_Sources})
target_compile_definitions(DEMO_PA10_TEST PRIVATE
    BUCK_PWM_TEST_PA10_ONLY=1 ADC_ONLY_TEST=$<BOOL:${ADC_ONLY_TEST}>)
target_link_libraries(DEMO_PA10_TEST PRIVATE stm32cubemx STM32_Drivers ${TOOLCHAIN_LINK_LIBRARIES})
target_link_options(DEMO_PA10_TEST PRIVATE "-Wl,-Map=DEMO_PA10_TEST.map")

# Explicit MANUALLY ARMED bench controller; never selected/built by default.
# Allows nominal VREF feedback only as an identified bench estimate. Still
# requires hardware_ready=1 and a separate START command after reset.
add_executable(DEMO_BUCK_CONTROL_TEST EXCLUDE_FROM_ALL ${DEMO_Application_Sources})
target_compile_definitions(DEMO_BUCK_CONTROL_TEST PRIVATE
    ADC_ONLY_TEST=0 BUCK_CONTROL_POWER_STAGE=1 BUCK_CONTROL_NOMINAL_BENCH=1)
target_link_libraries(DEMO_BUCK_CONTROL_TEST PRIVATE stm32cubemx STM32_Drivers ${TOOLCHAIN_LINK_LIBRARIES})
target_link_options(DEMO_BUCK_CONTROL_TEST PRIVATE "-Wl,-Map=DEMO_BUCK_CONTROL_TEST.map")

# User-requested automatic startup: qualified fresh feedback for 1 s, then
# soft-start once per reset. Running faults latch; no automatic retry.
# Kept separate from the default ADC preview and original manual bench target.
add_executable(DEMO_BUCK_CONTROL_AUTO EXCLUDE_FROM_ALL ${DEMO_Application_Sources})
target_compile_definitions(DEMO_BUCK_CONTROL_AUTO PRIVATE
    ADC_ONLY_TEST=0 BUCK_CONTROL_POWER_STAGE=1 BUCK_CONTROL_NOMINAL_BENCH=1
    BUCK_CONTROL_AUTO_START=1)
target_link_libraries(DEMO_BUCK_CONTROL_AUTO PRIVATE stm32cubemx STM32_Drivers ${TOOLCHAIN_LINK_LIBRARIES})
target_link_options(DEMO_BUCK_CONTROL_AUTO PRIVATE "-Wl,-Map=DEMO_BUCK_CONTROL_AUTO.map")
