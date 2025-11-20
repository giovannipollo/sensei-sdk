#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import time
import sys

PORT = "/dev/tty.usbmodem212401"

BAUD = 115200
RECONNECT_DELAY = 2.0   # seconds between reconnect attempts
READ_TIMEOUT = 0.5      # seconds: makes read non-blocking-ish

def open_serial(port, baud):
    """
    Try to open the serial port.
    Returns a serial.Serial instance or None if it fails.
    """
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=READ_TIMEOUT,
        )
        print(f"[INFO] Opened {port} at {baud} baud")
        return ser
    except serial.SerialException as e:
        print(f"[WARN] Cannot open {port}: {e}")
        return None

def read_loop():
    ser = None

    while True:
        # Ensure we have an open port
        if ser is None or not ser.is_open:
            ser = open_serial(PORT, BAUD)
            if ser is None:
                # Port not available: wait and retry
                time.sleep(RECONNECT_DELAY)
                continue

        try:
            # Non-blocking / timeout-based read
            line = ser.readline()  # returns b'' on timeout
            if line:
                # Decode safely, ignore bad bytes
                print(line.decode(errors="replace").rstrip())
        except (serial.SerialException, OSError) as e:
            # SerialException: cable unplugged, device gone, etc.
            # OSError: some OS-level error on the file descriptor
            print(f"[ERROR] UART error: {e}")
            # Clean up and go back to reconnect loop
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            print(f"[INFO] Waiting {RECONNECT_DELAY}s before retry")
            time.sleep(RECONNECT_DELAY)
        except KeyboardInterrupt:
            print("\n[INFO] Interrupted by user, exiting.")
            if ser is not None and ser.is_open:
                ser.close()
            sys.exit(0)

if __name__ == "__main__":
    read_loop()
