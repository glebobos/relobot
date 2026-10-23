cmake_minimum_required(VERSION 3.13)

set(RELOBOT_PICO_BOARD "kamod_rp2040" CACHE STRING "ReloBot hardware profile")
option(RELOBOT_INA226_BENCH "Disable real I2C hardware access" ON)

add_compile_definitions(RELOBOT_FIRMWARE_FAULT_HOOKS=1)
include("${CMAKE_CURRENT_LIST_DIR}/../pico_firmware/cmake/import.cmake")

project(picoros_ina226 C CXX ASM)

include("${RELOBOT_PICO_ROOT}/cmake/picoros.cmake")

relobot_add_firmware(picoros_ina226
    src/main.c
    src/ina226.c
    src/float32_codec.c
    src/bool_codec.c
)

target_include_directories(picoros_ina226 PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(picoros_ina226 PRIVATE hardware_i2c hardware_gpio hardware_watchdog)
target_compile_definitions(picoros_ina226 PRIVATE RELOBOT_INA226_BENCH=$<BOOL:${RELOBOT_INA226_BENCH}>)
target_compile_options(picoros_ina226 PRIVATE -Wall -Wextra -Werror)
