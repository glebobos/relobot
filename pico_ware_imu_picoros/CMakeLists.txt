cmake_minimum_required(VERSION 3.13)

set(RELOBOT_PICO_BOARD "pico" CACHE STRING "ReloBot hardware profile")
option(RELOBOT_IMU_BENCH "Disable real I2C hardware access" ON)

add_compile_definitions(RELOBOT_FIRMWARE_FAULT_HOOKS=1)
include("${CMAKE_CURRENT_LIST_DIR}/../pico_firmware/cmake/import.cmake")

project(picoros_imu C CXX ASM)

include("${RELOBOT_PICO_ROOT}/cmake/picoros.cmake")

set(SHARED_DRIVERS_DIR "${CMAKE_CURRENT_LIST_DIR}/../shared/drivers")
set(ICM20948_DIR "${SHARED_DRIVERS_DIR}/ICM20948")

set(IMU_SOURCES
    src/main.c
    src/imu_sensor.cpp
    src/imu_codec.c
    ../shared/float32_codec.c
)

if(NOT RELOBOT_IMU_BENCH)
    list(APPEND IMU_SOURCES
        "${ICM20948_DIR}/Arduino-ICM20948.cpp"
        "${ICM20948_DIR}/DataConverter.c"
        "${ICM20948_DIR}/Icm20948Augmented.c"
        "${ICM20948_DIR}/Icm20948AuxCompassAkm.c"
        "${ICM20948_DIR}/Icm20948AuxTransport.c"
        "${ICM20948_DIR}/Icm20948DataBaseControl.c"
        "${ICM20948_DIR}/Icm20948DataBaseDriver.c"
        "${ICM20948_DIR}/Icm20948DataConverter.c"
        "${ICM20948_DIR}/Icm20948Dmp3Driver.c"
        "${ICM20948_DIR}/Icm20948LoadFirmware.c"
        "${ICM20948_DIR}/Icm20948MPUFifoControl.c"
        "${ICM20948_DIR}/Icm20948SelfTest.c"
        "${ICM20948_DIR}/Icm20948Setup.c"
        "${ICM20948_DIR}/Icm20948Transport.c"
        "${ICM20948_DIR}/Message.c"
    )
endif()

relobot_add_firmware(picoros_imu ${IMU_SOURCES})

target_include_directories(picoros_imu PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../shared"
    "${SHARED_DRIVERS_DIR}"
    "${ICM20948_DIR}"
)
target_link_libraries(picoros_imu PRIVATE hardware_i2c hardware_gpio hardware_watchdog)
target_compile_definitions(picoros_imu PRIVATE RELOBOT_IMU_BENCH=$<BOOL:${RELOBOT_IMU_BENCH}>)
target_compile_options(picoros_imu PRIVATE -Wall -Wextra)

set_source_files_properties(
    src/main.c
    src/imu_sensor.cpp
    src/imu_codec.c
    ../shared/float32_codec.c
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror"
)
