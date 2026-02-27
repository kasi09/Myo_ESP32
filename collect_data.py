#!/usr/bin/env python3
"""
Myo ESP32 – Training Data Collector
====================================
Reads labeled CSV samples from the ESP32 over Serial and saves them to a file.

Usage:
  1. Install dependency:  pip install pyserial
  2. Flash Myo_Library.ino to your ESP32
  3. Adjust PORT below to match your system (e.g. COM3, /dev/ttyUSB0)
  4. Run:  python collect_data.py
  5. In the Serial Monitor (or another terminal) type:
       h      – write CSV header (do this once at the start)
       0-9    – record one labeled sample with that label
       s/m/l  – vibrate short/medium/long (optional feedback)

Output:
  myo_YYYYMMDD_HHMMSS.csv in the current directory
"""

import serial
import csv
import sys
from datetime import datetime

# ── Configuration ─────────────────────────────────────────────────────────────
PORT = "COM3"      # Windows: "COM3", Linux/Mac: "/dev/ttyUSB0" or "/dev/tty.usbserial-*"
BAUD = 115200
# ─────────────────────────────────────────────────────────────────────────────

OUTPUT_FILE = f"myo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"


def print_counts(label_counts):
    counts = "  ".join(f"[{k}]: {v}" for k, v in sorted(label_counts.items()))
    print(f"\rSamples – {counts}    ", end="", flush=True)


def main():
    print(f"Connecting to {PORT} at {BAUD} baud...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"Error: {e}")
        print("Check that PORT is correct and the ESP32 is connected.")
        sys.exit(1)

    print(f"Connected. Saving to: {OUTPUT_FILE}")
    print("Waiting for CSV header (send 'h' from Serial Monitor)...\n")

    writer = None
    label_counts = {}
    total = 0

    with open(OUTPUT_FILE, "w", newline="") as f:
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # CSV header line
                if line.startswith("label,"):
                    writer = csv.writer(f)
                    writer.writerow(line.split(","))
                    f.flush()
                    print(f"Header saved: {line[:60]}...")
                    print("Now send labeled samples (0-9) from the Serial Monitor.\n")

                # CSV data line (starts with a digit and contains commas)
                elif writer and "," in line and line[0].isdigit():
                    writer.writerow(line.split(","))
                    f.flush()
                    total += 1
                    lbl = line.split(",")[0]
                    label_counts[lbl] = label_counts.get(lbl, 0) + 1
                    print_counts(label_counts)

                # Non-CSV output from ESP32 (Pose:, Battery:, BLE connected, etc.)
                else:
                    print(f"\n[ESP32] {line}")
                    if writer and label_counts:
                        print_counts(label_counts)

        except KeyboardInterrupt:
            print(f"\n\nStopped. Saved {total} samples to '{OUTPUT_FILE}'")
            if label_counts:
                print("Label distribution:")
                for k, v in sorted(label_counts.items()):
                    print(f"  Label {k}: {v} samples")
            ser.close()


if __name__ == "__main__":
    main()
