# Copyright (c) 2025 ETH Zurich and University of Bologna
# SPDX-License-Identifier: Apache-2.0

target_sources(app PRIVATE
    sgp41_i2c.c
)

target_include_directories(app PRIVATE
    .
)
