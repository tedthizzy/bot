# Host build of firmware/core: the tests and the MCU simulator.
#
# A3 splits the firmware into a freestanding core and IDF glue precisely so
# this file can exist -- the same object code the ESP32-S3 runs is compiled by
# clang, sanitised, and driven by the golden vectors shared with the Python
# codec.  Nothing here includes an ESP-IDF header.
#
#   cmake -S firmware/host -B firmware/host/build
#   cmake --build firmware/host/build
#   ctest --test-dir firmware/host/build --output-on-failure
#
# `-DROVER_HOST_TEST=ON` is accepted for the spelling ARCHITECTURE 10 uses; the
# targets are built either way.

cmake_minimum_required(VERSION 3.16)
project(rover_host C)

option(ROVER_HOST_TEST "Build the freestanding core's host tests" ON)
option(ROVER_SANITIZE "Build under -fsanitize=address,undefined" ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

get_filename_component(ROVER_FIRMWARE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(ROVER_REPO_DIR "${ROVER_FIRMWARE_DIR}/.." ABSOLUTE)
set(ROVER_VECTORS "${ROVER_REPO_DIR}/tests/contract/serial_vectors.jsonl")

add_compile_options(-Wall -Wextra -Werror -g)
if(ROVER_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

add_library(rover_core STATIC
  "${ROVER_FIRMWARE_DIR}/core/rover_crc.c"
  "${ROVER_FIRMWARE_DIR}/core/rover_codec.c"
  "${ROVER_FIRMWARE_DIR}/core/rover_control.c"
  "${ROVER_FIRMWARE_DIR}/core/rover_safety.c"
  "${ROVER_FIRMWARE_DIR}/core/rover_core.c"
)
target_include_directories(rover_core PUBLIC
  "${ROVER_FIRMWARE_DIR}/core/include"
  "${ROVER_FIRMWARE_DIR}/core"
)

add_executable(mcu_sim "${ROVER_FIRMWARE_DIR}/sim/mcu_sim.c")
target_link_libraries(mcu_sim PRIVATE rover_core)
# ARCHITECTURE 10 calls the simulator `mcu-sim`, and that is the spelling
# rover_devtools.mcu_sim.SEARCH_PATHS looks for.  The CMake *target* keeps the
# C identifier so `cmake --build . --target mcu_sim` still works.
set_target_properties(mcu_sim PROPERTIES OUTPUT_NAME mcu-sim)

if(ROVER_HOST_TEST)
  enable_testing()
  add_executable(rover_host_tests
    "${ROVER_FIRMWARE_DIR}/test/test_main.c"
    "${ROVER_FIRMWARE_DIR}/test/test_vectors.c"
    "${ROVER_FIRMWARE_DIR}/test/test_core.c"
  )
  target_link_libraries(rover_host_tests PRIVATE rover_core)
  target_compile_definitions(rover_host_tests PRIVATE
    ROVER_VECTORS_PATH="${ROVER_VECTORS}")
  add_test(NAME rover_host_tests COMMAND rover_host_tests)
endif()
