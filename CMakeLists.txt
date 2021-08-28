cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(htu21
	htu21.c
)

pico_enable_stdio_usb(htu21 1)
pico_enable_stdio_uart(htu21 0)

pico_add_extra_outputs(htu21)

target_link_libraries(htu21 pico_stdlib hardware_i2c)
