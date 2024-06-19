# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/led.h)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_IS31FL3194 is31fl3194.c)