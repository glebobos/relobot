include_guard(GLOBAL)

option(RELOBOT_PICOROS_USB_POLL "Use cooperative USB serial receive with the pinned Zenoh-Pico" ON)
option(RELOBOT_PICOROS_CONNECTIVITY "Enable native Zenoh connectivity diagnostics" OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

set(PICOROS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(PICOROS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
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
set(Z_FEATURE_CONNECTIVITY 0 CACHE STRING "" FORCE)
if(RELOBOT_PICOROS_CONNECTIVITY)
    set(Z_FEATURE_CONNECTIVITY 1 CACHE STRING "" FORCE)
endif()
set(Z_TRANSPORT_LEASE 60000 CACHE STRING "" FORCE)
set(Z_TRANSPORT_LEASE_EXPIRE_FACTOR 2 CACHE STRING "" FORCE)

add_subdirectory("${PICOROS_SOURCE}" "${CMAKE_BINARY_DIR}/picoros")
if(RELOBOT_PICOROS_CONNECTIVITY)
    target_compile_definitions(picoros PRIVATE PICOROS_CONNECTIVITY=1)
endif()
if(RELOBOT_PICOROS_USB_POLL)
    target_compile_definitions(zenohpico_static PUBLIC PICOROS_USB_POLL=1)
endif()

target_include_directories(picoros PRIVATE "${RELOBOT_PICO_ROOT}/config")
target_include_directories(zenohpico_static PRIVATE "${RELOBOT_PICO_ROOT}/config")
target_link_libraries(zenohpico_static
    hardware_uart pico_stdlib pico_rand FreeRTOS-Kernel-Heap4
    tinyusb_host tinyusb_device tinyusb_board
)

if(NOT EXISTS "${RELOBOT_ROS_TYPES_PATH}/libmicroros.a")
    message(FATAL_ERROR "RELOBOT_ROS_TYPES_PATH must contain generated ROS headers and libmicroros.a")
endif()
add_library(relobot_ros_types STATIC IMPORTED GLOBAL)
set_target_properties(relobot_ros_types PROPERTIES
    IMPORTED_LOCATION "${RELOBOT_ROS_TYPES_PATH}/libmicroros.a"
    INTERFACE_INCLUDE_DIRECTORIES "${RELOBOT_ROS_TYPES_PATH}/include"
)

relobot_add_board()
add_library(relobot_picoros_runtime STATIC "${RELOBOT_PICO_ROOT}/src/picoros_runtime.c")
target_include_directories(relobot_picoros_runtime PUBLIC
    "${RELOBOT_PICO_ROOT}/include" "${RELOBOT_PICO_ROOT}/config"
)
target_link_libraries(relobot_picoros_runtime PUBLIC
    picoros pico_stdlib pico_rand FreeRTOS-Kernel-Heap4
    tinyusb_host tinyusb_device tinyusb_board
)
target_compile_options(relobot_picoros_runtime PRIVATE -Wall -Wextra -Werror)

function(relobot_add_firmware target)
    add_executable(${target} ${ARGN})
    if(RELOBOT_PICOROS_USB_POLL)
        target_sources(${target} PRIVATE
            "${RELOBOT_PICO_ROOT}/src/usb_serial/serial_rx.c"
            "${RELOBOT_PICO_ROOT}/src/usb_serial/serial_poll_link.c"
            "${RELOBOT_PICO_ROOT}/src/usb_serial/serial_poll_rp2040.c"
        )
        target_link_options(${target} PRIVATE
            -Wl,--wrap=_z_new_link_serial
            -Wl,--wrap=_z_serial_close
            -Wl,--wrap=_zp_unicast_read_task_fn
        )
    endif()
    target_link_libraries(${target} PRIVATE
        relobot_picoros_runtime relobot_board relobot_ros_types
    )
    pico_enable_stdio_usb(${target} 0)
    pico_enable_stdio_uart(${target} 0)
    pico_set_program_name(${target} "${target}")
    pico_add_extra_outputs(${target})
endfunction()
