# Copyright (c) 2025 ETH Zurich and University of Bologna
# SPDX-License-Identifier: Apache-2.0

macro(setup_custom_bsp)

# Board options
    if(CONFIG_BOARD_SENSEI_BASEV1)
         list(APPEND CUSTOM_BSP_SRCS "${GAP_CUSTOM_BSP}/bsp/boards/bsp_sensei_basev1.c")
         list(APPEND CUSTOM_BSP_COMPILE_OPTIONS "-D__GAP_BOARD_SENSEI_BASEV1__=1")
    endif()

    if(CONFIG_BOARD_SENSEI_SENSORV1)
        list(APPEND CUSTOM_BSP_SRCS "${GAP_CUSTOM_BSP}/bsp/boards/bsp_sensei_sensorv1.c")
        list(APPEND CUSTOM_BSP_COMPILE_OPTIONS "-D__GAP_BOARD_SENSEI_SENSORV1__=1")
   endif()


    if(CONFIG_BOARD_SENSEI_DEBUGV1)
        list(APPEND CUSTOM_BSP_SRCS "${GAP_CUSTOM_BSP}/bsp/boards/bsp_sensei_debugv1.c")
        list(APPEND CUSTOM_BSP_COMPILE_OPTIONS "-D__GAP_BOARD_SENSEI_DEBUGV1__=1")
    endif()

# Driver options
    if(CUSTOM_BSP_SRCS)
        message(STATUS "============================ Creating custom_bsp library")
        message(STATUS "${CUSTOM_BSP_PREFIX} Sources: ${CUSTOM_BSP_SRCS}")
        message(STATUS "${CUSTOM_BSP_PREFIX} Options: ${CUSTOM_BSP_COMPILE_OPTIONS}")
        add_library(custom_bsp STATIC ${CUSTOM_BSP_SRCS})
        if ("${TARGET_LINKER_LANGUAGE}" STREQUAL "CXX")
            set_target_properties(custom_bsp PROPERTIES LINKER_LANGUAGE CXX)
        endif()
        target_include_directories(custom_bsp PUBLIC ${GAP_CUSTOM_BSP})
        target_link_libraries(custom_bsp PRIVATE pmsis_implem)
        target_compile_options(custom_bsp PUBLIC ${CUSTOM_BSP_COMPILE_OPTIONS})
    endif()
endmacro()