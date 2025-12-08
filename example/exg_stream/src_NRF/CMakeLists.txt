# Copyright (c) 2024 ETH Zurich
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Enable shields
set(SHIELD "SENSEI_ExGShield SENSEI_PPGShield")

set(CONF_FILE prj.conf)
set(OVERLAY_CONFIG "child_image/hci_ipc.conf")


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
project(exg_stream)


target_sources(app PRIVATE
    main.c
    bluetooth.c
    bsp/pwr_bsp.c
    ble_appl.c
    ads_appl.c
    ads_spi.c
    board_streaming.c
    ppg_appl.c
    i2c_helpers.c
    imu_appl.c
    lis2duxs12_sensor.c
    lis2duxs12_reg.c
    state_machine.c
    mic_appl.c
    sync_streaming.c
)
target_include_directories(app PRIVATE
    .
)
