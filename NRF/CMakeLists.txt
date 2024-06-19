# Copyright (c) 2025 ETH Zurich and University of Bologna
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SENSEI system)
add_subdirectory_ifdef(CONFIG_SENSEI drivers)

target_include_directories_ifdef(CONFIG_SENSEI app PRIVATE
    system
)