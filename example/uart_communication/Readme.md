<!--
Copyright (c) 2024 ETH Zurich and University of Bologna
SPDX-License-Identifier: Apache-2.0
-->

# Hello World - UART Communication Example

This example demonstrates basic bidirectional UART communication between the GAP9 and nRF5340 processors on the SENSEI platform.

## Overview

- **GAP9**: Sends "Hello from GAP9! Count: X" messages every second over UART
- **nRF5340**: Receives messages from GAP9, logs them via RTT, and sends acknowledgments back

## Hardware Setup

The UART connection uses:
- GAP9 UART0 <-> nRF UART3 (`uart_gap`)
- Baud rate: 115200

## Building

### Prerequisites

1. Set up the SENSEI SDK environment:
   ```bash
   export SENSEI_SDK_ROOT=/path/to/sensei-sdk
   ```

2. Ensure you have the GAP SDK and Zephyr SDK installed.

### Build GAP9 Firmware

```bash
cd example/hello_world/src_GAP9
mkdir -p build && cd build
cmake .. -DBOARD=sensei_basev1
make
```

### Build nRF5340 Firmware

```bash
cd example/hello_world/src_NRF
west build -b nrf5340_senseiv1_cpuapp
```

## Flashing

### Flash nRF5340

```bash
west flash
```

### Flash GAP9

Use the GAP SDK tools to flash the GAP9 firmware.

## Viewing Output

Use SEGGER RTT Viewer or J-Link RTT to view the nRF5340 log output:

```
[INF] ===========================================
[INF]   nRF <-> GAP9 Hello World Example
[INF] ===========================================
[INF] Board: nrf5340_senseiv1_cpuapp
[INF] GAP UART device ready
[INF] GAP UART callback configured
[INF] UART RX enabled
[INF] Power started - GAP9 should be booting...
[INF] Waiting for messages from GAP9...
[INF] [RX #1] Hello from GAP9! Count: 1
[INF] [RX #2] Hello from GAP9! Count: 2
...
```

## Files

```
hello_world/
├── src_GAP9/
│   ├── CMakeLists.txt    # GAP9 build configuration
│   ├── main.c            # GAP9 UART transmitter
│   └── sdk.config        # GAP9 SDK configuration
├── src_NRF/
│   ├── bsp/
│   │   ├── pwr_bsp.c     # Power management BSP
│   │   └── pwr_bsp.h     # Power management BSP header
│   ├── CMakeLists.txt    # Zephyr build configuration
│   ├── main.c            # nRF UART receiver
│   └── prj.conf          # Zephyr project configuration
└── Readme.md             # This file
```

## Communication Protocol

This example uses simple newline-terminated ASCII strings:

- **GAP9 -> nRF**: `Hello from GAP9! Count: X\n`
- **nRF -> GAP9**: `nRF ACK #X\n`

For more complex protocols, see the `yolov5` example which uses binary framing with special byte sequences.

## Troubleshooting

1. **No output from GAP9**: Ensure power management is correctly configured and GAP9 is booting.
2. **UART not receiving**: Check that the baud rate matches on both sides (115200).
3. **Buffer allocation failures**: Increase `CONFIG_HEAP_MEM_POOL_SIZE` in `prj.conf`.
