# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../ubxlib)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(celllocate_wifi_example)

FILE(GLOB app_sources src/*.c)
FILE(GLOB head_sources src/*.h)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE ${head_sources})
