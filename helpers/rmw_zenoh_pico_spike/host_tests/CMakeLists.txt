cmake_minimum_required(VERSION 3.16)
project(picoros_serial_executor_experiment C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
option(PICOROS_CONNECTIVITY_TESTS "Test built-in connectivity through the Pico-ROS adapter" OFF)

set(ZENOH_PICO_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../build/picoros/thirdparty/zenoh-pico"
    CACHE PATH "Pinned Zenoh-Pico checkout populated by build_picoros.sh")
get_filename_component(zenoh_source_path "${ZENOH_PICO_SOURCE_DIR}" REALPATH)
execute_process(COMMAND git -c "safe.directory=${zenoh_source_path}"
    -C "${zenoh_source_path}" rev-parse HEAD
    OUTPUT_VARIABLE zenoh_revision OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE revision_result)
if(NOT revision_result EQUAL 0 OR
   NOT zenoh_revision STREQUAL "0eb81272682a526b4bcf5a850c16ac74bbc6c73e")
    message(FATAL_ERROR "Populate the pinned checkout with build_picoros.sh first")
endif()

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(ZP_PLATFORM linux CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_SERIAL 1 CACHE STRING "" FORCE)
set(Z_FEATURE_CONNECTIVITY 0 CACHE STRING "" FORCE)
if(PICOROS_CONNECTIVITY_TESTS)
    set(Z_FEATURE_CONNECTIVITY 1 CACHE STRING "" FORCE)
    set(Z_FEATURE_UNSTABLE_API 1 CACHE STRING "" FORCE)
endif()
add_subdirectory("${zenoh_source_path}" zenoh-pico)
find_package(Threads REQUIRED)

add_executable(serial_executor_silence serial_executor_silence.c)
target_link_libraries(serial_executor_silence PRIVATE zenohpico::static Threads::Threads)
target_compile_options(serial_executor_silence PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_options(serial_executor_silence PRIVATE -Wl,--wrap=_z_serial_read)
enable_testing()
add_test(NAME serial_executor_silence COMMAND serial_executor_silence)
set_tests_properties(serial_executor_silence PROPERTIES TIMEOUT 10)

set(relobot_usb_source "${CMAKE_CURRENT_LIST_DIR}/../../../pico_firmware/src/usb_serial")
add_library(serial_rx_experiment STATIC "${relobot_usb_source}/serial_rx.c")
target_include_directories(serial_rx_experiment PUBLIC "${relobot_usb_source}")
target_link_libraries(serial_rx_experiment PUBLIC zenohpico::static)
target_compile_options(serial_rx_experiment PRIVATE -Wall -Wextra -Werror)
foreach(experiment IN ITEMS serial_rx_test serial_rx_executor)
    add_executable(${experiment} ${experiment}.c)
    target_link_libraries(${experiment} PRIVATE serial_rx_experiment)
    target_compile_options(${experiment} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME ${experiment} COMMAND ${experiment})
    set_tests_properties(${experiment} PROPERTIES TIMEOUT 10)
endforeach()

add_executable(serial_poll_link_test serial_poll_link_test.c
    "${relobot_usb_source}/serial_poll_link.c")
target_link_libraries(serial_poll_link_test PRIVATE serial_rx_experiment)
target_compile_options(serial_poll_link_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_options(serial_poll_link_test PRIVATE
    -Wl,--wrap=_z_new_link_serial -Wl,--wrap=_z_serial_close
    -Wl,--wrap=_zp_unicast_read_task_fn -Wl,--wrap=_z_read_serial)
add_test(NAME serial_poll_link_test COMMAND serial_poll_link_test)
set_tests_properties(serial_poll_link_test PROPERTIES TIMEOUT 10)

if(PICOROS_CONNECTIVITY_TESTS)
    add_library(picoros_interface_test STATIC ../build/picoros/src/picoros.c)
    target_include_directories(picoros_interface_test PUBLIC ../build/picoros/src)
    target_compile_definitions(picoros_interface_test PUBLIC PICOROS_CONNECTIVITY=1)
    target_link_libraries(picoros_interface_test PUBLIC zenohpico::static)
    add_executable(picoros_connectivity_test picoros_connectivity_test.c
        ../firmware_picoros/common/picoros_connectivity.c)
    target_include_directories(picoros_connectivity_test PRIVATE ../firmware_picoros/common)
    target_link_libraries(picoros_connectivity_test PRIVATE picoros_interface_test)
    target_compile_options(picoros_connectivity_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME picoros_connectivity_test COMMAND picoros_connectivity_test)
    set_tests_properties(picoros_connectivity_test PROPERTIES TIMEOUT 15)

    add_executable(picoros_connectivity_client_reconnect picoros_connectivity_client_reconnect.c
        ../firmware_picoros/common/picoros_connectivity.c)
    target_include_directories(picoros_connectivity_client_reconnect PRIVATE ../firmware_picoros/common)
    target_link_libraries(picoros_connectivity_client_reconnect PRIVATE picoros_interface_test)
    target_compile_options(picoros_connectivity_client_reconnect PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME picoros_connectivity_client_gap
        COMMAND picoros_connectivity_client_reconnect --expect-missing-reopen-event)
    set_tests_properties(picoros_connectivity_client_gap PROPERTIES TIMEOUT 20 LABELS known-limitation)
endif()
