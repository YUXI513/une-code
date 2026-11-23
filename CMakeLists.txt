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

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/oled.c
    Core/Src/buck_pwm.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Separate diagnostic firmware: only PA10 switches; PB13 stays low.
# Build explicitly with: cmake --build --preset Debug --target DEMO_PA10_TEST
# EXCLUDE_FROM_ALL keeps the normal DEMO build unchanged.
get_target_property(DEMO_Application_Sources ${CMAKE_PROJECT_NAME} SOURCES)
add_executable(DEMO_PA10_TEST EXCLUDE_FROM_ALL ${DEMO_Application_Sources})
target_compile_definitions(DEMO_PA10_TEST PRIVATE BUCK_PWM_TEST_PA10_ONLY=1)
target_link_libraries(DEMO_PA10_TEST PRIVATE stm32cubemx STM32_Drivers ${TOOLCHAIN_LINK_LIBRARIES})
target_link_options(DEMO_PA10_TEST PRIVATE "-Wl,-Map=DEMO_PA10_TEST.map")
