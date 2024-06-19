# Copyright (c) 2022 Philip Schilk
# SPDX-License-Identifier: Apache-2.0

target_sources(app PRIVATE
    src/max77654.c
)
target_include_directories(app PRIVATE
    inc
)
