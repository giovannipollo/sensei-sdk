# Copyright (c) 2025 ETH Zurich and University of Bologna
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

set(SHIELD SENSEI_DebugBoard_v1)

# Setup SENSEI SDK
set(SENSEI_SDK_ROOT $ENV{SENSEI_SDK_ROOT})
if(NOT SENSEI_SDK_ROOT)
    message(FATAL_ERROR "SENSEI_SDK_ROOT is not defined")
endif()

list(APPEND EXTRA_ZEPHYR_MODULES
    ${SENSEI_SDK_ROOT}/NRF
)

# Configure the project
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(debug_test)

target_sources(app PRIVATE
    main.c
    bsp/pwr_bsp.c
)

target_include_directories(app PRIVATE
    .
)