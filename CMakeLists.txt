# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(button)

FILE(GLOB app_sources src/*c)
target_sources(app PRIVATE ${app_sources})

set(ENV{CROSS_COMPILE} "/opt/local/bin/arm-none-eabi-")
