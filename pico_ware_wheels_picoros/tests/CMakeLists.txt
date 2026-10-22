cmake_minimum_required(VERSION 3.13)
project(wheels_tests C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
enable_testing()

foreach(stub_header IN ITEMS FreeRTOS.h task.h picoros.h tusb.h
        pico/stdlib.h pico/bootrom.h hardware/gpio.h hardware/pwm.h
        hardware/sync.h hardware/watchdog.h sensor_msgs/msg/joint_state.h)
    get_filename_component(stub_directory "${CMAKE_CURRENT_BINARY_DIR}/stubs/${stub_header}" DIRECTORY)
    file(MAKE_DIRECTORY "${stub_directory}")
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/stubs/${stub_header}" "#include \"platform_stubs.h\"\n")
endforeach()

add_executable(wheels_bench_test
    wheels_test.c
    ../../shared/pid_controller.c
)
target_include_directories(wheels_bench_test PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/stubs"
    .
    ../src
    ../../shared
    ../../pico_firmware/include
)
target_compile_definitions(wheels_bench_test PRIVATE
    RELOBOT_WHEELS_BENCH=1
    UNIT_TEST=1
)
target_compile_options(wheels_bench_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(wheels_bench_test PRIVATE m)

add_executable(wheels_motor_test
    wheels_test.c
    ../../shared/pid_controller.c
)
target_include_directories(wheels_motor_test PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/stubs"
    .
    ../src
    ../../shared
    ../../pico_firmware/include
)
target_compile_definitions(wheels_motor_test PRIVATE
    RELOBOT_WHEELS_BENCH=0
    UNIT_TEST=1
)
target_compile_options(wheels_motor_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(wheels_motor_test PRIVATE m)

foreach(test_case IN ITEMS defaults commands control bench faults cdc)
    add_test(NAME wheels_bench_${test_case} COMMAND wheels_bench_test ${test_case})
endforeach()

foreach(test_case IN ITEMS defaults commands control motor faults cdc)
    add_test(NAME wheels_motor_${test_case} COMMAND wheels_motor_test ${test_case})
endforeach()
