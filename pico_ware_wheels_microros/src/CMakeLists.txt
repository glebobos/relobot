cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_wheels_microros C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(/project/libmicroros)

add_executable(pico_wheels_microros
    main.cpp
    pid_controller.c
    pico_uart_transport.c
)

target_link_libraries(pico_wheels_microros
    pico_stdlib
    hardware_pwm
    hardware_watchdog
    microros
)

target_include_directories(pico_wheels_microros PUBLIC
    /project/libmicroros/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

pico_enable_stdio_usb(pico_wheels_microros 1)
pico_enable_stdio_uart(pico_wheels_microros 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(pico_wheels_microros)
