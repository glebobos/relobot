cmake_minimum_required(VERSION 3.13)
project(ina226_tests C)
set(CMAKE_C_STANDARD 11)
enable_testing()

foreach(stub_header IN ITEMS FreeRTOS.h task.h picoros.h tusb.h
        pico/stdlib.h pico/bootrom.h hardware/gpio.h hardware/i2c.h
        hardware/sync.h hardware/watchdog.h)
    get_filename_component(stub_directory "${CMAKE_CURRENT_BINARY_DIR}/stubs/${stub_header}" DIRECTORY)
    file(MAKE_DIRECTORY "${stub_directory}")
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/stubs/${stub_header}" "#include \"platform_stubs.h\"\n")
endforeach()

add_executable(ina226_bench_test
    ina226_test.c
    ../src/ina226.c
)
target_include_directories(ina226_bench_test PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/stubs"
    .
    ../src
    ../../pico_firmware/include
)
target_compile_definitions(ina226_bench_test PRIVATE
    RELOBOT_INA226_BENCH=1
)
target_compile_options(ina226_bench_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(ina226_bench_test PRIVATE m)

add_executable(ina226_sensor_test
    ina226_test.c
    ../src/ina226.c
)
target_include_directories(ina226_sensor_test PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/stubs"
    .
    ../src
    ../../pico_firmware/include
)
target_compile_definitions(ina226_sensor_test PRIVATE
    RELOBOT_INA226_BENCH=0
)
target_compile_options(ina226_sensor_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(ina226_sensor_test PRIVATE m)

foreach(test_case IN ITEMS voltage dock bench faults cdc)
    add_test(NAME ina226_bench_${test_case} COMMAND ina226_bench_test ${test_case})
endforeach()

foreach(test_case IN ITEMS voltage dock sensor faults cdc)
    add_test(NAME ina226_sensor_${test_case} COMMAND ina226_sensor_test ${test_case})
endforeach()
