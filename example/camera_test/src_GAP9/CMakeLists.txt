# Copyright (c) 2025 ETH Zurich and University of Bologna
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19)

###############################################################################
# Panel Control
###############################################################################
set(TARGET_NAME "main")


###############################################################################
# CMake pre initialization
###############################################################################
set(GAP_CUSTOM_BSP $ENV{SENSEI_SDK_ROOT}/GAP9)

set(CONFIG_GAP_SDK_HOME $ENV{GAP_SDK_HOME})
include($ENV{GAP_SDK_HOME}/utils/cmake/setup.cmake)

set(ISP_KER_PATH $ENV{TILER_ISP_KERNEL_PATH})
set(MODEL_SRC ISPModel.c)

set(TARGET_SRCS test.c
                ISPKernels.c
                ${ISP_KER_PATH}/ISP_BasicKernels.c
                ${ISP_KER_PATH}
)

list(APPEND TARGET_COMPILE_OPTIONS  -mno-memcpy
                                    -fno-tree-loop-distribute-patterns
                                    -Wno-maybe-uninitialized
                                    -Wno-unused-but-set-variable
                                    -Wno-unused-variable
                                    -I$ENV{GAP_SDK_HOME}/tools/autotiler_v3/Emulation
                                    -I$ENV{GAP_SDK_HOME}/tools/autotiler_v3/Autotiler
                                    -I$ENV{GAP_SDK_HOME}/tools/autotiler_v3/ISP_Libraries
                                    -I$ENV{GAP_SDK_HOME}/tools/autotiler_v3/ISP_Generators/
                                    -I$ENV{GAP_SDK_HOME}/tools/autotiler_v3/CNN_Libraries_fp16/
                                    -I${CMAKE_BINARY_DIR}
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(${TARGET_NAME} C ASM)
add_executable(${TARGET_NAME} ${TARGET_SRCS})

target_compile_definitions(${TARGET_NAME} PUBLIC "-DTRACE_CSI2")

target_print_header(${TARGET_NAME})

###############################################################################
# CMake Project Options
###############################################################################
target_print_option(STATUS ${TARGET_NAME} ${Cyan} "HM0360 Format" "${CONFIG_DRIVER_HM0360_FORMAT}" "${Cyan}")
target_print_option(STATUS ${TARGET_NAME} ${Cyan} "HM0360 Width " "${CONFIG_DRIVER_HM0360_FORMAT_WIDTH}" "${Cyan}")
target_print_option(STATUS ${TARGET_NAME} ${Cyan} "HM0360 Hight " "${CONFIG_DRIVER_HM0360_FORMAT_HEIGHT}" "${Cyan}")

if (CONFIG_SAVING_PICTURE)
    target_print_option(STATUS ${TARGET_NAME} ${Cyan} "Saving picture" "Enable" "${Green}")
    target_compile_options(${TARGET_NAME} PRIVATE "-DSAVE_PIC -DPPM_FILE=${CMAKE_CURRENT_SOURCE_DIR}img.ppm -DJPEG_FILE=${CMAKE_CURRENT_SOURCE_DIR}img.jpg")
    else()
    target_print_option(STATUS ${TARGET_NAME} ${Cyan} "Saving picture" "Disable" "${Red}")
endif()

if (CONFIG_APP_LOG)
    target_print_option(STATUS ${TARGET_NAME} ${Cyan} "Logs" "Enable" "${Green}")
    target_compile_options(${TARGET_NAME} PRIVATE "-DCONFIG_APP_LOG=1")
else()
    target_print_option(STATUS ${TARGET_NAME} ${Cyan} "Logs" "Disable" "${Red}")
endif()

if(TARGET_COMPILE_OPTIONS)
    target_compile_options(${TARGET_NAME} PUBLIC ${TARGET_COMPILE_OPTIONS})
endif()


target_print_compile_options(STATUS ${TARGET_NAME} ${Cyan})
target_print_footer(${TARGET_NAME})

###############################################################################
# CMake post initialization
###############################################################################
setupos(${TARGET_NAME})
