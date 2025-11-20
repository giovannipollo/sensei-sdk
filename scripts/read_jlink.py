import pylink
import time
import sys

DEVICE_NAME = "NRF5340_XXAA_APP"  # adjust if needed
CHANNEL = 0                        # RTT channel

def read_rtt(device=DEVICE_NAME, channel=CHANNEL):
    jlink = pylink.JLink()

    # Try to connect repeatedly if device is busy (e.g., during west flash)
    while True:
        try:
            jlink.open()
            jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
            jlink.connect(device)
            break
        except Exception as e:
            print(f"Waiting for device... ({e})")
            time.sleep(1)

    print("Connected to RTT. Listening for logs...\n")

    # Start RTT
    jlink.rtt_start()
    
    try:
        while True:
            data = jlink.rtt_read(channel, 4096)  # read big chunks
            if data:
                print(data.decode(errors="ignore"), end="")
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopping RTT listener.")
    finally:
        jlink.close()


if __name__ == "__main__":
    try:
        read_rtt()
    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)
