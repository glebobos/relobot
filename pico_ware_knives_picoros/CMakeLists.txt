cmake_minimum_required(VERSION 3.13)

set(RELOBOT_PICO_BOARD "xiao_rp2040" CACHE STRING "ReloBot hardware profile")
include("${CMAKE_CURRENT_LIST_DIR}/../pico_firmware/cmake/import.cmake")

project(picoros_knives C CXX ASM)

include("${RELOBOT_PICO_ROOT}/cmake/picoros.cmake")

relobot_add_firmware(picoros_knives
    src/main.c
    src/pid_controller.c
    src/float32_codec.c
)

target_include_directories(picoros_knives PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(picoros_knives PRIVATE hardware_pwm hardware_gpio)
