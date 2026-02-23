cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_imu_microros C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(/project/libmicroros)

add_executable(pico_imu_microros
    main.cpp
    pico_uart_transport.c
    ICM20948/Arduino-ICM20948.cpp
    ICM20948/DataConverter.c
    ICM20948/Icm20948Augmented.c
    ICM20948/Icm20948AuxCompassAkm.c
    ICM20948/Icm20948AuxTransport.c
    ICM20948/Icm20948DataBaseControl.c
    ICM20948/Icm20948DataBaseDriver.c
    ICM20948/Icm20948DataConverter.c
    ICM20948/Icm20948Dmp3Driver.c
    ICM20948/Icm20948LoadFirmware.c
    ICM20948/Icm20948MPUFifoControl.c
    ICM20948/Icm20948SelfTest.c
    ICM20948/Icm20948Setup.c
    ICM20948/Icm20948Transport.c
    ICM20948/Message.c
)

target_link_libraries(pico_imu_microros
    pico_stdlib
    hardware_i2c
    microros
)

target_include_directories(pico_imu_microros PUBLIC
    /project/libmicroros/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/ICM20948
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_imu_microros 1)
pico_enable_stdio_uart(pico_imu_microros 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_imu_microros)
