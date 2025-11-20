#!/usr/bin/env python3
import os
import subprocess
import time
import pylink
import sys
import shutil
import re

# ===== CONFIGURATION =====
WEST_BUILD_DIR = "../example/exg_stream/src_NRF/"
BOARD_NAME = "nrf5340_senseiv1_cpuapp"
DEVICE_NAME = "NRF5340_XXAA_APP"
RTT_CHANNEL = 0
RTT_READ_SIZE = 4096
CONDA_ENV = "nrf"
LOG_FILE = "build_flash_rtt.log"  # combined log file

# Environment variables
ENV_VARS = {
    "SENSEI_SDK_ROOT": "/Users/giovanni/git/sensei-sdk-new",
    "ZEPHYR_BASE": "/opt/nordic/ncs/v2.6.0/zephyr",
}
# =========================

def run_command(cmd, cwd=None, log_file=None):
    """Run shell command inside conda environment with ENV_VARS set and log output live to terminal and optionally to a file."""
    conda_exe = shutil.which("conda")
    if conda_exe is None:
        raise RuntimeError("conda executable not found in PATH")

    full_cmd = [conda_exe, "run", "-n", CONDA_ENV] + cmd
    env = os.environ.copy()
    env.update(ENV_VARS)

    # open log file only if provided
    f = open(log_file, "a") if log_file else None

    # ANSI/control stripping helper
    ansi_re = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")
    control_re = re.compile(r"[\x00-\x08\x0b\x0c\x0e-\x1f\x7f]")

    def _clean(s: str) -> str:
        s = ansi_re.sub("", s)
        s = control_re.sub("", s)
        return s

    try:
        process = subprocess.Popen(full_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, cwd=cwd, env=env, bufsize=1, universal_newlines=True)
        # bufsize=1 + universal_newlines=True enables line-buffered output
        for line in process.stdout:
            clean_line = _clean(line)
            print(clean_line, end="", flush=True)       # print live to terminal (cleaned)
            if f:
                f.write(clean_line)
                f.flush()                        # write live to file
        process.wait()
        if process.returncode != 0:
            raise RuntimeError(f"Command failed: {' '.join(cmd)}")
    finally:
        if f:
            f.close()


def wait_for_device(jlink, device=DEVICE_NAME):
    while True:
        try:
            jlink.open()
            jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
            jlink.connect(device)
            break
        except Exception as e:
            print(f"Waiting for device... ({e})")
            time.sleep(1)

def stream_rtt(jlink, channel=RTT_CHANNEL, read_size=RTT_READ_SIZE, log_file=None):
    jlink.rtt_start()
    try:
        # open log file (or dummy) and stream RTT output, cleaning ANSI/control chars
        with open(log_file, "a") if log_file else subprocess.DEVNULL as f:
            ansi_re = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")
            control_re = re.compile(r"[\x00-\x08\x0b\x0c\x0e-\x1f\x7f]")

            def _clean(s: str) -> str:
                s = ansi_re.sub("", s)
                s = control_re.sub("", s)
                return s

            while True:
                data = jlink.rtt_read(channel, read_size)
                if data:
                    text = bytes(data).decode(errors="ignore")
                    clean_text = _clean(text)
                    print(clean_text, end="")
                    if log_file:
                        f.write(clean_text)
                        f.flush()
                else:
                    time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopping RTT listener.")
    finally:
        jlink.close()

def main():
    try:
        # Clear previous log
        if os.path.exists(LOG_FILE):
            os.remove(LOG_FILE)

        print("=== WEST BUILD ===")
        run_command(["west", "build", "-b", BOARD_NAME], cwd=WEST_BUILD_DIR, log_file=LOG_FILE)

        print("\n=== WEST FLASH ===")
        run_command(["west", "flash"], cwd=WEST_BUILD_DIR, log_file=LOG_FILE)

        print("\n=== RTT LOGGER ===")
        time.sleep(0.5)  # 500 ms delay to let app initialize RTT
        jlink = pylink.JLink()
        wait_for_device(jlink)
        print("Connected to RTT. Listening for logs...\n")
        stream_rtt(jlink, log_file=LOG_FILE)

    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
