# Copyright (c) 2025 ETH Zurich and University of Bologna
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SENSEI_PWR pwr)

add_subdirectory(bsp)
add_subdirectory(third_party)