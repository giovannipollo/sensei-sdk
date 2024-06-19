# Copyright (c) 2025 ETH Zurich and University of Bologna
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_NRF5340_SENSEIv1_CPUAPP)
board_runner_args(jlink "--device=nrf5340_xxaa_app" "--speed=4000")
board_runner_args(pyocd "--target=nrf5340_cpuapp" "--frequency=4000000")
endif()

if(CONFIG_BOARD_NRF5340_SENSEIv1_CPUNET)
board_runner_args(jlink "--device=nrf5340_xxaa_net" "--speed=4000")
board_runner_args(pyocd "--target=nrf5340_cpunet" "--frequency=4000000")
endif()

include(${ZEPHYR_BASE}/boards/common/nrfjprog.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)