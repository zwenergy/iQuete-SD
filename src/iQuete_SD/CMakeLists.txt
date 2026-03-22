cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 11)

set(PICO_PLATFORM rp2040)
set(BOARD pico_sdk)

include(pico_sdk_import.cmake)

set(PROJECT IQUE)

project(${PROJECT} C CXX ASM)

pico_sdk_init()

add_subdirectory(../no-OS-FatFS-SD build)

add_compile_options( -Ofast -Wall )

# For boards with crystals which take a bit longer to stablize.
add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)

add_executable(${PROJECT} main.c)
pico_generate_pio_header(${PROJECT} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio ${CMAKE_CURRENT_LIST_DIR}/get_input.pio)

# Can leave these off for silent mode:
add_compile_definitions(USE_PRINTF USE_DBG_PRINTF)
add_compile_definitions(USE_PRINTF)

# Add the standard library and FatFS/SPI to the build
target_link_libraries(${PROJECT} 
    pico_stdlib
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
)

pico_enable_stdio_usb(${PROJECT} 1)
pico_enable_stdio_uart(${PROJECT} 0)        

pico_add_extra_outputs(${PROJECT})

target_sources(${PROJECT} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/main.c)

target_link_libraries(${PROJECT} pico_stdlib hardware_pio)
