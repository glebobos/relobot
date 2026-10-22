cmake_minimum_required(VERSION 3.13)

set(RELOBOT_PICO_BOARD "kamod_rp2040" CACHE STRING "ReloBot hardware profile")
option(RELOBOT_WHEELS_BENCH "Disable all motor and encoder GPIO access" ON)
if(NOT RELOBOT_WHEELS_BENCH AND NOT RELOBOT_PICO_BOARD STREQUAL "xiao_rp2040")
    message(FATAL_ERROR "Motor mode is only defined for xiao_rp2040; use bench mode on KAmod")
endif()
add_compile_definitions(RELOBOT_FIRMWARE_FAULT_HOOKS=1)
include("${CMAKE_CURRENT_LIST_DIR}/../pico_firmware/cmake/import.cmake")

project(picoros_wheels C CXX ASM)

include("${RELOBOT_PICO_ROOT}/cmake/picoros.cmake")

relobot_add_firmware(picoros_wheels
    src/main.c
    ../shared/pid_controller.c
    src/joint_state_codec.c
    ../shared/float32_codec.c
)

target_include_directories(picoros_wheels PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../shared"
)
target_link_libraries(picoros_wheels PRIVATE hardware_pwm hardware_gpio hardware_watchdog)
target_compile_definitions(picoros_wheels PRIVATE RELOBOT_WHEELS_BENCH=$<BOOL:${RELOBOT_WHEELS_BENCH}>)
target_compile_options(picoros_wheels PRIVATE -Wall -Wextra -Werror)
