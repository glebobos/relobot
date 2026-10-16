cmake_minimum_required(VERSION 3.13)

include("${CMAKE_CURRENT_LIST_DIR}/../../../pico_firmware/cmake/import.cmake")

project(picoros_usb_spike C CXX ASM)

set(RELOBOT_ROS_TYPES_PATH "${CMAKE_CURRENT_LIST_DIR}/../build/libmicroros"
    CACHE PATH "Generated ROS message support")
set(RELOBOT_PICOROS_CONNECTIVITY ON)
include("${RELOBOT_PICO_ROOT}/cmake/picoros.cmake")

if(NOT RELOBOT_PICOROS_USB_POLL)
    message(FATAL_ERROR "The USB validation application requires cooperative USB serial receive")
endif()

relobot_add_firmware(picoros_usb_connectivity
    main.c
    common/float32_codec.c
    common/command_probe.c
    diagnostics/transport_probe.c
)
target_sources(picoros_usb_connectivity PRIVATE common/picoros_connectivity.c)
target_include_directories(picoros_usb_connectivity PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/common
    ${CMAKE_CURRENT_LIST_DIR}/diagnostics
)
target_compile_options(picoros_usb_connectivity PRIVATE -Wall -Wextra -Werror)
target_compile_definitions(picoros_usb_connectivity PRIVATE
    PICOROS_DIAGNOSTICS=1 PICOROS_TRANSPORT_PROBE=1 PICOROS_CONNECTIVITY=1
)
target_link_options(picoros_usb_connectivity PRIVATE
    -Wl,--wrap=_z_executor_spin -Wl,--wrap=_z_usb_uart_getc
)
