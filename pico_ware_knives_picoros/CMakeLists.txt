cmake_minimum_required(VERSION 3.13)

set(RELOBOT_PICO_BOARD "kamod_rp2040" CACHE STRING "ReloBot hardware profile")
option(RELOBOT_KNIVES_BENCH "Disable all actuator and encoder GPIO access" ON)
if(NOT RELOBOT_KNIVES_BENCH AND NOT RELOBOT_PICO_BOARD STREQUAL "xiao_rp2040")
    message(FATAL_ERROR "Motor mode is only defined for xiao_rp2040; use bench mode on KAmod")
endif()
add_compile_definitions(RELOBOT_FIRMWARE_FAULT_HOOKS=1)
include("${CMAKE_CURRENT_LIST_DIR}/../pico_firmware/cmake/import.cmake")

project(picoros_knives C CXX ASM)

include("${RELOBOT_PICO_ROOT}/cmake/picoros.cmake")

relobot_add_firmware(picoros_knives
    src/main.c
    src/pid_controller.c
    src/float32_codec.c
    src/command_state.c
)

target_include_directories(picoros_knives PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(picoros_knives PRIVATE hardware_pwm hardware_gpio hardware_watchdog)
target_compile_definitions(picoros_knives PRIVATE RELOBOT_KNIVES_BENCH=$<BOOL:${RELOBOT_KNIVES_BENCH}>)
target_compile_options(picoros_knives PRIVATE -Wall -Wextra -Werror)
