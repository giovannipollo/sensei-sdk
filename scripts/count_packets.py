#!/usr/bin/env python3
"""
Simple packet counter - reads packets from serial and counts by header type.

Usage: python count_packets.py [port] [--duration SECS]
"""

import serial
import serial.tools.list_ports
import sys
import time

# Serial settings
DEFAULT_BAUD = 1000000
READ_TIMEOUT = 0.1

# Packet headers and sizes
EEG_HEADER = 0x55
MIC_HEADER = 0xAA
EEG_PACKET_SIZE = 234
MIC_PACKET_SIZE = 131

# Commands
CMD_START_MERGED = 31
CMD_STOP_MERGED = 32
DEFAULT_CONFIG = bytes([6, 0, 2, 4, 0, 13, 10])


def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return None
    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device} - {port.description}")
    if len(ports) == 1:
        return ports[0].device
    try:
        return ports[int(input("Select port number: "))].device
    except (ValueError, IndexError):
        return None


def main():
    # Parse args
    args = sys.argv[1:]
    duration = None
    if "--duration" in args:
        idx = args.index("--duration")
        args.remove("--duration")
        duration = float(args.pop(idx))
    
    port = args[0] if args else list_ports()
    if not port:
        return

    # Connect
    print(f"Opening {port} at {DEFAULT_BAUD} baud...")
    ser = serial.Serial(port=port, baudrate=DEFAULT_BAUD, timeout=READ_TIMEOUT)
    
    # Start streaming
    ser.write(bytes([CMD_START_MERGED]))
    time.sleep(0.1)
    ser.write(DEFAULT_CONFIG)
    print("Started streaming...")
    time.sleep(1)

    # Counters
    eeg_count = 0
    mic_count = 0
    unknown_count = 0
    buffer = bytearray()
    start_time = time.time()
    last_print = start_time

    try:
        while True:
            if duration and (time.time() - start_time) >= duration:
                break

            data = ser.read(1024)
            if data:
                buffer.extend(data)

            # Process buffer
            while len(buffer) >= 2:
                header = buffer[0]

                if header == EEG_HEADER and len(buffer) >= EEG_PACKET_SIZE:
                    if buffer[EEG_PACKET_SIZE - 1] == MIC_HEADER:
                        eeg_count += 1
                        del buffer[:EEG_PACKET_SIZE]
                    else:
                        buffer.pop(0)
                        unknown_count += 1
                elif header == MIC_HEADER and len(buffer) >= MIC_PACKET_SIZE:
                    if buffer[MIC_PACKET_SIZE - 1] == EEG_HEADER:
                        mic_count += 1
                        del buffer[:MIC_PACKET_SIZE]
                    else:
                        buffer.pop(0)
                        unknown_count += 1
                elif header not in (EEG_HEADER, MIC_HEADER):
                    buffer.pop(0)
                    unknown_count += 1
                else:
                    break  # Wait for more data

            # Print stats every second
            now = time.time()
            if now - last_print >= 1.0:
                last_print = now
                elapsed = now - start_time
                print(f"[{elapsed:5.1f}s] EEG: {eeg_count:6d}  Mic: {mic_count:6d}  Unknown: {unknown_count}")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ser.write(bytes([CMD_STOP_MERGED]))
        time.sleep(0.1)
        ser.close()

    # Final stats
    elapsed = time.time() - start_time
    print(f"\n{'='*50}")
    print(f"Final counts ({elapsed:.1f}s):")
    print(f"  EEG packets:     {eeg_count:6d} (header 0x{EEG_HEADER:02X})")
    print(f"  Mic packets:     {mic_count:6d} (header 0x{MIC_HEADER:02X})")
    print(f"  Unknown bytes:   {unknown_count:6d}")
    print(f"{'='*50}")


if __name__ == "__main__":
    main()
