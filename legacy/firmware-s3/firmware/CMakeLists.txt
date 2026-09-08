cmake_minimum_required(VERSION 3.16)

# ESP-IDF v5.5.5, ESP32-S3-DevKitC-1-N8R8. Build in the container:
#   firmware/docker/build.sh          release, no console anywhere (A37)
#   firmware/docker/build.sh debug    console on USB-Serial/JTAG
set(PROJECT_VER "0.1.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rover_fw)
