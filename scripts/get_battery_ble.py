import argparse
import serial

def open_serial_port(port: str, baudrate: int) -> serial.Serial:
    """Open a serial port for BLE communication."""
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
    return ser
    
def main():
    parser = argparse.ArgumentParser(
        description="Get battery status from SENSEI board via BLE",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("--port", type=str, required=True, help="Serial port for BLE communication")
    parser.add_argument("--baudrate", type=int, default=9600, help="Baudrate for serial communication (default: 9600)")
    args = parser.parse_args()

    try:
        ser = open_serial_port(port=args.port, baudrate=args.baudrate)
        print("Successfully opened serial port:", args.port)
    except serial.SerialException as e:
        print("Error opening serial port:", e)
        return

    # Send command to get battery status
    ser.write(bytes([17]))  

    # Read response
    response = ser.read(7)

    if len(response) == 7:
        print("Is battery charging:", response[1])
        print("Battery level (%):", response[4])

    ser.close()

if __name__ == "__main__":
    main()
