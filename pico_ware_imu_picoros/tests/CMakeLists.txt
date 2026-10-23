cmake_minimum_required(VERSION 3.13)
project(imu_tests C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
enable_testing()

foreach(stub_header IN ITEMS FreeRTOS.h task.h picoros.h tusb.h
        pico/stdlib.h pico/bootrom.h hardware/gpio.h hardware/i2c.h
        hardware/sync.h hardware/watchdog.h sensor_msgs/msg/imu.h)
    get_filename_component(stub_directory "${CMAKE_CURRENT_BINARY_DIR}/stubs/${stub_header}" DIRECTORY)
    file(MAKE_DIRECTORY "${stub_directory}")
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/stubs/${stub_header}" "#include \"platform_stubs.h\"\n")
endforeach()

add_executable(imu_bench_test
    imu_test.c
    ../src/imu_sensor.cpp
)
target_include_directories(imu_bench_test PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/stubs"
    .
    ../src
    ../../shared
    ../../pico_firmware/include
)
target_compile_definitions(imu_bench_test PRIVATE
    RELOBOT_IMU_BENCH=1
    UNIT_TEST=1
)
target_compile_options(imu_bench_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(imu_bench_test PRIVATE m)

add_executable(imu_sensor_test
    imu_test.c
    ../src/imu_sensor.cpp
)
target_include_directories(imu_sensor_test PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/stubs"
    .
    ../src
    ../../shared
    ../../pico_firmware/include
)
target_compile_definitions(imu_sensor_test PRIVATE
    RELOBOT_IMU_BENCH=0
    UNIT_TEST=1
)
target_compile_options(imu_sensor_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(imu_sensor_test PRIVATE m)

foreach(test_case IN ITEMS defaults bench faults cdc staleness)
    add_test(NAME imu_bench_${test_case} COMMAND imu_bench_test ${test_case})
endforeach()

foreach(test_case IN ITEMS defaults sensor faults cdc staleness)
    add_test(NAME imu_sensor_${test_case} COMMAND imu_sensor_test ${test_case})
endforeach()
