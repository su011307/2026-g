# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.18)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
list(PREPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)
list(REMOVE_DUPLICATES PICO_BOARD_HEADER_DIRS)
set(PICO_BOARD_HEADER_DIRS "${PICO_BOARD_HEADER_DIRS}" CACHE STRING "Custom board headers" FORCE)
if(NOT DEFINED PICO_BOARD OR PICO_BOARD STREQUAL "pico2")
    set(PICO_BOARD wavegen_rp2350b CACHE STRING "Board type" FORCE)
endif()

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(wavegen C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add CMSIS-DSP. CMSISCORE points the library at the CMSIS-Core headers
# supplied by the Pico SDK.
include(FetchContent)
set(CMSISCORE ${PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core)
set(DISABLEFLOAT16 ON CACHE BOOL "Disable CMSIS-DSP float16 kernels" FORCE)
FetchContent_Declare(cmsis_dsp
        GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP.git
        GIT_TAG v1.17.0
        GIT_SHALLOW TRUE
        SOURCE_SUBDIR Source
)
FetchContent_MakeAvailable(cmsis_dsp)

# ARM GCC 15.2 ICEs on several unused CMSIS Q31 translation units at -O1 and
# above. Keep the third-party target at its known-good optimized-debug level;
# the application target below is still built with -O2.
target_compile_options(CMSISDSP PRIVATE -Og)

# Add executable. Default name is the project name, version 0.1

add_executable(wavegen
        main.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/app/wavegen.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/app/adc9226.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/app/ad9959.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/app/ads8688.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/app/get_raw_adc9226.cpp

        src/initialize.cpp
        src/signal.cpp
        src/utility.cpp

        lib/tset2/src/display.cpp
        lib/st7789/src/driver_st7789.c
        lib/st7789/port/rp2350/driver_st7789_interface_rp2350.c
        lib/st7789/port/rp2350/driver_st7789_rp2350.c
)

# Keep the application path optimized even in an IDE Debug configuration.
target_compile_options(wavegen PRIVATE -O2)

pico_set_program_name(wavegen "wavegen")
pico_set_program_version(wavegen "0.1")

# Generate PIO header
pico_generate_pio_header(wavegen ${CMAKE_CURRENT_LIST_DIR}/pio/transmit.pio)
pico_generate_pio_header(wavegen ${CMAKE_CURRENT_LIST_DIR}/pio/read.pio)
pico_generate_pio_header(wavegen ${CMAKE_CURRENT_LIST_DIR}/pio/ads8688.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(wavegen 0)
pico_enable_stdio_usb(wavegen 0)

# Add the standard library to the build
target_link_libraries(wavegen
        pico_stdlib
        cmsis_core
        CMSISDSP
)

# Add the standard include files to the build
target_include_directories(wavegen PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src

        ${CMAKE_CURRENT_LIST_DIR}/lib/tset2/src
        ${CMAKE_CURRENT_LIST_DIR}/lib/tset2/include
        ${CMAKE_CURRENT_LIST_DIR}/lib/st7789/src
        ${CMAKE_CURRENT_LIST_DIR}/lib/st7789/interface
        ${CMAKE_CURRENT_LIST_DIR}/lib/st7789/port/rp2350
        ${CMAKE_CURRENT_LIST_DIR}/lib/st7789/test
)

# Add any user requested libraries
target_link_libraries(wavegen
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_pio
)

pico_add_extra_outputs(wavegen)
