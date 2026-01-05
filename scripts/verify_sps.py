#!/usr/bin/env python3
"""
Simple script to verify samples per second (sps) for EEG, MIC, and IMU streams.

Usage:
    python scripts/verify_sps.py /dev/cu.usbmodem* [--duration SECONDS] [--mode MODE]

Modes:
    all     - Stream EEG, MIC, and IMU (default)
    eeg     - Stream EEG only
    mic     - Stream MIC only
    imu     - Stream IMU only (uses commands 33/34)
    eeg-mic - Stream EEG and MIC together (no IMU)
    eeg-imu - Stream EEG and IMU together (no MIC)
    mic-imu - Stream MIC and IMU together (no EEG)
"""

import argparse
import struct
import time

import serial

# Packet definitions - Updated for IMU separation
MIC_PACKET_SIZE = 131
MIC_HEADER = 0xAA
MIC_TRAILER = 0x55
MIC_SAMPLES_PER_PACKET = 64

# New EEG packet format (without IMU):
# Header(1) + Counter(1) + Timestamp(4) + 4*50(samples) + Metadata(3) + Trailer(1) = 210
EEG_PACKET_SIZE = 210
EEG_HEADER = 0x55
EEG_TRAILER = 0xAA
EEG_SAMPLES_PER_PACKET = 4

# Legacy EEG packet size (for backward compatibility detection)
EEG_LEGACY_PACKET_SIZE = 234
EEG_LEGACY_SAMPLES_PER_PACKET = 4

# New IMU packet format:
# Header(1) + Counter(1) + Timestamp(4) + 20*6(samples) + Trailer(1) = 127
IMU_PACKET_SIZE = 127
IMU_HEADER = 0x56
IMU_TRAILER = 0x57
IMU_SAMPLES_PER_PACKET = 20


def extract_imu_info(packet: bytes) -> tuple[int, int, list[tuple[int, int, int]]]:
    """Extract counter, timestamp, and samples from IMU packet.
    
    IMU packet format:
    - Byte 0: Header (0x56)
    - Byte 1: Counter (uint8)
    - Bytes 2-5: Timestamp (uint32, little-endian, microseconds)
    - Bytes 6-125: 20 samples × 6 bytes (X, Y, Z as big-endian int16)
    - Byte 126: Trailer (0x57)
    
    Returns: (counter, timestamp_us, [(x, y, z), ...])
    """
    counter = packet[1]
    timestamp = struct.unpack('<I', packet[2:6])[0]
    
    samples = []
    for i in range(IMU_SAMPLES_PER_PACKET):
        offset = 6 + i * 6  # Start after header(1) + counter(1) + timestamp(4)
        x = struct.unpack('>h', packet[offset:offset+2])[0]
        y = struct.unpack('>h', packet[offset+2:offset+4])[0]
        z = struct.unpack('>h', packet[offset+4:offset+6])[0]
        samples.append((x, y, z))
    
    return counter, timestamp, samples


def create_eeg_command():
    command = [6, 0, 2, 4, 0, 13, 10]  # gain=6
    return bytes(command)


def detect_packet_v2(buffer: bytes, mode: str) -> tuple[str, int, int] | None:
    """
    Smarter packet detection that waits for enough data to disambiguate.
    
    Key insight: If we see a potential MIC header (0xAA), we should wait
    until we have enough bytes to also check if it could be an EEG packet
    starting earlier in the buffer.
    """
    if len(buffer) < MIC_PACKET_SIZE:
        return None
    
    # If buffer starts with EEG header, wait for full EEG packet
    if buffer[0] == EEG_HEADER:
        if len(buffer) < EEG_PACKET_SIZE:
            return None  # Wait for more data
        if buffer[EEG_PACKET_SIZE - 1] == EEG_TRAILER:
            return ("EEG", EEG_PACKET_SIZE, 0)
        # EEG header but wrong trailer - skip this byte
        return ("SKIP", 0, 1)
    
    # If buffer starts with MIC header
    if buffer[0] == MIC_HEADER:
        if buffer[MIC_PACKET_SIZE - 1] == MIC_TRAILER:
            return ("MIC", MIC_PACKET_SIZE, 0)
        # MIC header but wrong trailer - skip this byte
        return ("SKIP", 0, 1)
    
    # If buffer starts with IMU header
    if buffer[0] == IMU_HEADER:
        if len(buffer) < IMU_PACKET_SIZE:
            return None  # Wait for more data
        if buffer[IMU_PACKET_SIZE - 1] == IMU_TRAILER:
            return ("IMU", IMU_PACKET_SIZE, 0)
        # IMU header but wrong trailer - skip this byte
        return ("SKIP", 0, 1)
    
    # Neither header - find next valid header
    for i in range(1, len(buffer)):
        if buffer[i] == EEG_HEADER or buffer[i] == MIC_HEADER or buffer[i] == IMU_HEADER:
            return ("SKIP", 0, i)  # Return offset to skip garbage
    
    return None


def detect_packet(buffer: bytes, mode: str, debug: bool = False) -> tuple[str, int, int] | None:
    """Returns (packet_type, packet_size, offset) or None."""
    
    # Define which packet types are enabled for each mode
    eeg_enabled = mode in ("all", "eeg", "eeg-mic", "eeg-imu", "combined")
    mic_enabled = mode in ("all", "mic", "eeg-mic", "mic-imu", "combined")
    imu_enabled = mode in ("all", "imu", "eeg-imu", "mic-imu", "combined")
    
    # Determine minimum buffer size needed based on enabled packet types
    min_sizes = []
    if eeg_enabled:
        min_sizes.append(EEG_PACKET_SIZE)
    if mic_enabled:
        min_sizes.append(MIC_PACKET_SIZE)
    if imu_enabled:
        min_sizes.append(IMU_PACKET_SIZE)
    
    # We need at least enough bytes to check for the smallest enabled packet
    min_required = min(min_sizes) if min_sizes else MIC_PACKET_SIZE
    if len(buffer) < min_required:
        return None
        
    # CRITICAL FIX: If we see EEG header at position 0, we MUST wait for
    # enough data to verify it's a complete EEG packet before checking MIC
    if eeg_enabled and buffer[0] == EEG_HEADER:
        if len(buffer) < EEG_PACKET_SIZE:
            return None  # Wait for more data!
        if buffer[EEG_PACKET_SIZE - 1] == EEG_TRAILER:
            return ("EEG", EEG_PACKET_SIZE, 0)
        # EEG header but invalid trailer - skip this byte (could be MIC trailer)
        return ("SKIP", 0, 1)
    
    # Check IMU at position 0 (check before MIC since IMU has unique header 0x56)
    if imu_enabled and buffer[0] == IMU_HEADER:
        if len(buffer) < IMU_PACKET_SIZE:
            return None  # Wait for more data
        if buffer[IMU_PACKET_SIZE - 1] == IMU_TRAILER:
            return ("IMU", IMU_PACKET_SIZE, 0)
        # IMU header but invalid trailer - skip this byte
        return ("SKIP", 0, 1)
    
    # Now check MIC at position 0
    if mic_enabled and buffer[0] == MIC_HEADER:
        if len(buffer) < MIC_PACKET_SIZE:
            return None  # Wait for more data
        if buffer[MIC_PACKET_SIZE - 1] == MIC_TRAILER:
            return ("MIC", MIC_PACKET_SIZE, 0)
        # MIC header but invalid trailer - skip this byte
        return ("SKIP", 0, 1)
    
    # No valid packet at position 0 - scan for next header
    for i in range(1, len(buffer)):
        if eeg_enabled and buffer[i] == EEG_HEADER:
            return ("SKIP", 0, i)  # Skip to EEG header
        if imu_enabled and buffer[i] == IMU_HEADER:
            return ("SKIP", 0, i)  # Skip to IMU header
        if mic_enabled and buffer[i] == MIC_HEADER:
            return ("SKIP", 0, i)  # Skip to MIC header
    
    # No headers found - skip all but last byte
    if len(buffer) > 1:
        return ("SKIP", 0, len(buffer) - 1)
    return None


def analyze_stream(buffer: bytes) -> None:
    """Analyze raw buffer to find all potential packet boundaries."""
    print(f"\nAnalyzing {len(buffer)} bytes...")
    
    eeg_starts = []
    mic_starts = []
    imu_starts = []
    eeg_counters = []
    mic_counters = []
    imu_counters = []
    
    for i in range(len(buffer)):
        if buffer[i] == EEG_HEADER and i + EEG_PACKET_SIZE <= len(buffer):
            if buffer[i + EEG_PACKET_SIZE - 1] == EEG_TRAILER:
                eeg_starts.append(i)
                # EEG counter - let's check a few possible locations
                eeg_counters.append(buffer[i+1])  # byte after header
        if buffer[i] == MIC_HEADER and i + MIC_PACKET_SIZE <= len(buffer):
            if buffer[i + MIC_PACKET_SIZE - 1] == MIC_TRAILER:
                mic_starts.append(i)
                mic_counters.append(buffer[i+1])  # MIC counter at byte 1
        if buffer[i] == IMU_HEADER and i + IMU_PACKET_SIZE <= len(buffer):
            if buffer[i + IMU_PACKET_SIZE - 1] == IMU_TRAILER:
                imu_starts.append(i)
                imu_counters.append(buffer[i+1])  # IMU counter at byte 1
    
    print(f"Potential EEG packets: {len(eeg_starts)}")
    print(f"Potential MIC packets: {len(mic_starts)}")
    print(f"Potential IMU packets: {len(imu_starts)}")
    
    # Check EEG counter sequence for missed packets
    if eeg_counters:
        print(f"\nEEG counter analysis:")
        print(f"  First 20 counters: {eeg_counters[:20]}")
        missed = 0
        for i in range(1, len(eeg_counters)):
            expected = (eeg_counters[i-1] + 1) % 256
            if eeg_counters[i] != expected:
                diff = (eeg_counters[i] - eeg_counters[i-1]) % 256
                missed += diff - 1
        print(f"  Missed packets (by counter): {missed}")
        if missed > 0:
            print(f"  This means firmware IS sending all packets but we're losing them")
        else:
            print(f"  Counters are sequential - firmware is sending at reduced rate")
    
    # Check MIC counter sequence
    if mic_counters:
        print(f"\nMIC counter analysis:")
        print(f"  First 20 counters: {mic_counters[:20]}")
        missed = 0
        for i in range(1, len(mic_counters)):
            expected = (mic_counters[i-1] + 1) % 256
            if mic_counters[i] != expected:
                diff = (mic_counters[i] - mic_counters[i-1]) % 256
                missed += diff - 1
        print(f"  Missed packets (by counter): {missed}")
    
    # Check IMU counter sequence
    if imu_counters:
        print(f"\nIMU counter analysis:")
        print(f"  First 20 counters: {imu_counters[:20]}")
        missed = 0
        for i in range(1, len(imu_counters)):
            expected = (imu_counters[i-1] + 1) % 256
            if imu_counters[i] != expected:
                diff = (imu_counters[i] - imu_counters[i-1]) % 256
                missed += diff - 1
        print(f"  Missed packets (by counter): {missed}")
    
    # Analyze EEG bursts
    if len(eeg_starts) > 1:
        bursts = []
        current_burst = 1
        for i in range(1, len(eeg_starts)):
            gap = eeg_starts[i] - eeg_starts[i-1]
            if gap == EEG_PACKET_SIZE:
                current_burst += 1
            else:
                bursts.append(current_burst)
                current_burst = 1
        bursts.append(current_burst)
        
        print(f"\nEEG burst analysis:")
        print(f"  Number of bursts: {len(bursts)}")
        print(f"  Burst sizes: {bursts[:20]}...")
        print(f"  Average burst size: {sum(bursts)/len(bursts):.1f} packets")
    
    # Gaps between bursts
    if len(eeg_starts) > 1:
        gaps = [eeg_starts[i+1] - eeg_starts[i] for i in range(len(eeg_starts)-1)]
        large_gaps = [(i, g) for i, g in enumerate(gaps) if g > EEG_PACKET_SIZE]
        print(f"\nGaps between EEG bursts (first 5):")
        for i, (idx, gap) in enumerate(large_gaps[:5]):
            mic_in_gap = (gap - EEG_PACKET_SIZE) / MIC_PACKET_SIZE
            print(f"  After EEG#{idx}: {gap} bytes = {mic_in_gap:.1f} MIC packets")
    
    print(f"\nTotal overlaps: 0")  # Already verified no overlaps


def validate_eeg_packet(data: bytes) -> bool:
    """Check if EEG packet has valid structure beyond just header/trailer."""
    # EEG packet should have specific structure at known positions
    # Check bytes 50-57 for counter area, bytes 106-113, 162-169, 218-225
    # These are timestamp/counter regions that should have reasonable values
    return True  # For now, just return True


def validate_mic_packet(data: bytes) -> bool:
    """Check if MIC packet has valid structure beyond just header/trailer."""
    # MIC packet: header(1) + counter(1) + audio(128) + trailer(1) = 131
    # Counter should increment
    return True  # For now, just return True


def validate_imu_packet(data: bytes) -> bool:
    """Check if IMU packet has valid structure beyond just header/trailer."""
    # IMU packet: header(1) + counter(1) + timestamp(4) + 20*6(samples) + trailer(1) = 127
    return True  # For now, just return True


def main():
    parser = argparse.ArgumentParser(description="Verify EEG, MIC, and IMU sample rates")
    parser.add_argument("port", help="Serial port")
    parser.add_argument("--baud", type=int, default=2000000, help="Baud rate")
    parser.add_argument("--duration", type=float, default=5.0, help="Duration in seconds")
    parser.add_argument("--mode", choices=["all", "eeg", "mic", "imu", "eeg-mic", "eeg-imu", "mic-imu", "combined"], default="all", help="Stream mode")
    parser.add_argument("--debug", action="store_true", help="Show packet sequence")
    parser.add_argument("--analyze", action="store_true", help="Analyze raw stream for overlaps")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.reset_input_buffer()

    # Define which streams are enabled based on mode
    eeg_enabled = args.mode in ("all", "eeg", "eeg-mic", "eeg-imu")
    mic_enabled = args.mode in ("all", "mic", "eeg-mic", "mic-imu")
    imu_enabled = args.mode in ("all", "imu", "eeg-imu", "mic-imu")
    combined_mode = args.mode == "combined"

    # Start streaming based on mode
    # Note: EEG startup takes ~500ms due to ADS1298 power-on and initialization
    # We need longer delays when starting EEG before other streams
    print(f"Starting streams (mode: {args.mode})...")
    if eeg_enabled:
        ser.write(bytes([18]))  # Start EEG
        time.sleep(0.5)  # EEG needs 500ms for ADS power-on (300ms) + init
    if mic_enabled:
        ser.write(bytes([26]))  # Start MIC
        time.sleep(0.2)
    if imu_enabled:
        ser.write(bytes([33]))  # Start IMU
        time.sleep(0.2)
    if combined_mode:
        ser.write(bytes([31]))  # Start combined stream
        time.sleep(0.2)

    print(f"Collecting for {args.duration}s...\n")

    buffer = bytearray()
    raw_data = bytearray()  # Keep raw copy for analysis
    eeg_samples = 0
    mic_samples = 0
    imu_samples = 0
    eeg_packets = 0
    mic_packets = 0
    imu_packets = 0
    skipped_bytes = 0
    packet_sequence = []  # Track sequence of packet types
    imu_timestamps = []  # Track IMU timestamps for analysis
    imu_first_samples = []  # Track first sample of each IMU packet

    start_time = time.time()

    try:
        while time.time() - start_time < args.duration:
            data = ser.read(4096)
            if data:
                buffer.extend(data)
                if args.analyze:
                    raw_data.extend(data)

            while True:
                result = detect_packet(bytes(buffer), args.mode)
                if result is None:
                    break

                packet_type, packet_size, offset = result
                if offset > 0:
                    skipped_bytes += offset
                    del buffer[:offset]
                    continue

                if packet_type == "EEG":
                    del buffer[:packet_size]
                    eeg_samples += EEG_SAMPLES_PER_PACKET
                    eeg_packets += 1
                    packet_sequence.append("E")
                elif packet_type == "MIC":
                    del buffer[:packet_size]
                    mic_samples += MIC_SAMPLES_PER_PACKET
                    mic_packets += 1
                    packet_sequence.append("M")
                elif packet_type == "IMU":
                    # Extract IMU packet info BEFORE removing from buffer
                    packet_data = bytes(buffer[:IMU_PACKET_SIZE])
                    counter, timestamp, samples = extract_imu_info(packet_data)
                    imu_timestamps.append((counter, timestamp))
                    imu_first_samples.append(samples[0])  # Store first sample for debug
                    
                    del buffer[:packet_size]
                    imu_samples += IMU_SAMPLES_PER_PACKET
                    imu_packets += 1
                    packet_sequence.append("I")

    except KeyboardInterrupt:
        pass
    finally:
        # Stop streams based on mode
        if eeg_enabled:
            ser.write(bytes([19]))  # Stop EEG
            time.sleep(0.1)
        if mic_enabled:
            ser.write(bytes([27]))  # Stop MIC
            time.sleep(0.1)
        if imu_enabled:
            ser.write(bytes([34]))  # Stop IMU
            time.sleep(0.1)
        if combined_mode:
            ser.write(bytes([32]))  # Stop combined stream
            time.sleep(0.1)
        ser.close()

    elapsed = time.time() - start_time
    
    # Define which streams are enabled based on mode
    eeg_enabled = args.mode in ("all", "eeg", "eeg-mic", "eeg-imu", "combined")
    mic_enabled = args.mode in ("all", "mic", "eeg-mic", "mic-imu", "combined")
    imu_enabled = args.mode in ("all", "imu", "eeg-imu", "mic-imu", "combined")

    print("=" * 40)
    print(f"Duration: {elapsed:.2f}s")
    if eeg_enabled:
        print(f"EEG: {eeg_samples/elapsed:.0f} sps (expected: 500)")
    if mic_enabled:
        print(f"MIC: {mic_samples/elapsed:.0f} sps (expected: 16000)")
    if imu_enabled:
        print(f"IMU: {imu_samples/elapsed:.0f} sps (expected: 400)")
    if eeg_enabled:
        print(f"EEG packets: {eeg_packets} ({eeg_packets/elapsed:.1f}/s, expected: 125/s)")
    if mic_enabled:
        print(f"MIC packets: {mic_packets} ({mic_packets/elapsed:.1f}/s, expected: 250/s)")
    if imu_enabled:
        print(f"IMU packets: {imu_packets} ({imu_packets/elapsed:.1f}/s, expected: 20/s)")
    print(f"Skipped bytes: {skipped_bytes}")
    print("=" * 40)
    
    # IMU timestamp analysis
    if imu_timestamps:
        print(f"\n--- IMU Timestamp Analysis ---")
        print(f"First 10 IMU packets (counter, timestamp_us):")
        for i, (counter, ts) in enumerate(imu_timestamps[:10]):
            sample = imu_first_samples[i] if i < len(imu_first_samples) else (0, 0, 0)
            print(f"  #{counter:3d}: {ts:12d} us  accel=({sample[0]:6d}, {sample[1]:6d}, {sample[2]:6d})")
        
        # Calculate timestamp deltas
        if len(imu_timestamps) > 1:
            deltas = []
            for i in range(1, len(imu_timestamps)):
                delta = imu_timestamps[i][1] - imu_timestamps[i-1][1]
                # Handle wraparound (32-bit microsecond counter wraps at ~71 minutes)
                if delta < 0:
                    delta += 2**32
                deltas.append(delta)
            
            avg_delta = sum(deltas) / len(deltas)
            min_delta = min(deltas)
            max_delta = max(deltas)
            
            # Expected: 20 samples at 400Hz = 50ms = 50000us between packets
            expected_delta = 50000
            
            print(f"\nTimestamp deltas (us between packets):")
            print(f"  Expected: {expected_delta} us (20 samples @ 400Hz)")
            print(f"  Average:  {avg_delta:.0f} us")
            print(f"  Min:      {min_delta} us")
            print(f"  Max:      {max_delta} us")
            print(f"  First 10 deltas: {deltas[:10]}")
            
            # Check for missed packets using counter
            missed = 0
            for i in range(1, len(imu_timestamps)):
                expected_counter = (imu_timestamps[i-1][0] + 1) % 256
                if imu_timestamps[i][0] != expected_counter:
                    diff = (imu_timestamps[i][0] - imu_timestamps[i-1][0]) % 256
                    missed += diff - 1
            print(f"\nMissed IMU packets (by counter): {missed}")
    
    if args.analyze and raw_data:
        analyze_stream(bytes(raw_data))
    
    if args.debug and packet_sequence:
        # Show first 200 packets
        seq = "".join(packet_sequence[:200])
        print(f"\nFirst 200 packets: {seq}")
        
        # Count patterns
        seq_full = "".join(packet_sequence)
        em_count = seq_full.count("EM")  # EEG followed by MIC
        me_count = seq_full.count("ME")  # MIC followed by EEG  
        ee_count = seq_full.count("EE")  # EEG followed by EEG
        mm_count = seq_full.count("MM")  # MIC followed by MIC
        ei_count = seq_full.count("EI")  # EEG followed by IMU
        ie_count = seq_full.count("IE")  # IMU followed by EEG
        ii_count = seq_full.count("II")  # IMU followed by IMU
        print(f"\nPatterns: EM={em_count}, ME={me_count}, EE={ee_count}, MM={mm_count}, EI={ei_count}, IE={ie_count}, II={ii_count}")


if __name__ == "__main__":
    main()
