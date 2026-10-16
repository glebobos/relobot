cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD "pico" CACHE STRING "RP2040 board")

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../build/zenoh-pico/examples/rpi_pico/FreeRTOS_Kernel_import.cmake)

project(rmw_zenoh_pico_spike C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(WITH_RPI_PICO ON CACHE BOOL "" FORCE)
set(Z_FEATURE_LINK_TCP 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_UDP_MULTICAST 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_UDP_UNICAST 0 CACHE STRING "" FORCE)
set(Z_FEATURE_SCOUTING_UDP 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_SERIAL 1 CACHE STRING "" FORCE)
set(Z_FEATURE_UNSTABLE_API 1 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_SERIAL_USB 1 CACHE STRING "" FORCE)
set(Z_TRANSPORT_LEASE 60000 CACHE STRING "" FORCE)
set(Z_TRANSPORT_LEASE_EXPIRE_FACTOR 2 CACHE STRING "" FORCE)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../build/zenoh-pico zenoh-pico)

file(GLOB RMW_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/../build/rmw_zenoh_pico/rmw_zenoh_pico/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/../build/rmw_zenoh_pico/rmw_zenoh_pico/src/zenoh_pico/*.c
)

add_library(rmw_zenoh_pico STATIC ${RMW_SOURCES})
target_include_directories(rmw_zenoh_pico PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/../build/rmw_zenoh_pico/rmw_zenoh_pico/include
    ${CMAKE_CURRENT_LIST_DIR}/../build/libmicroros/include
)
target_compile_definitions(rmw_zenoh_pico PUBLIC
    RMW_ZENOH_PICO_TRANSPORT_SERIAL
    ZENOH_RPI_PICO
)
target_link_libraries(rmw_zenoh_pico PUBLIC
    zenohpico::lib
)

target_include_directories(zenohpico_static PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
target_link_libraries(zenohpico_static
    hardware_uart
    pico_stdlib
    pico_rand
    FreeRTOS-Kernel-Heap4
    tinyusb_host
    tinyusb_device
    tinyusb_board
)

add_library(kamod_led STATIC kamod_led.c)
pico_generate_pio_header(kamod_led ${CMAKE_CURRENT_LIST_DIR}/kamod_ws2812.pio)
target_link_libraries(kamod_led PUBLIC pico_stdlib hardware_pio hardware_clocks)

add_executable(rmw_zenoh_pico_spike main.c)
target_include_directories(rmw_zenoh_pico_spike PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/../build/libmicroros/include
)
target_link_directories(rmw_zenoh_pico_spike PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../build/libmicroros
)
target_link_libraries(rmw_zenoh_pico_spike
    -Wl,--start-group
    microros
    rmw_zenoh_pico
    -Wl,--end-group
    pico_stdlib
    pico_rand
    FreeRTOS-Kernel-Heap4
    tinyusb_host
    tinyusb_device
    tinyusb_board
    kamod_led
)

pico_enable_stdio_usb(rmw_zenoh_pico_spike 0)
pico_enable_stdio_uart(rmw_zenoh_pico_spike 0)
pico_set_program_name(rmw_zenoh_pico_spike "rmw_zenoh_pico_spike")
target_compile_definitions(rmw_zenoh_pico_spike PRIVATE
    USBD_PRODUCT="rmw_zenoh_pico_spike"
    USBD_MANUFACTURER="ReloBot"
)
pico_add_extra_outputs(rmw_zenoh_pico_spike)
