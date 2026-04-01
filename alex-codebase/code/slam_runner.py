"""
slam_runner.py
==============
Main SLAM runner for CG2111A Final Project.

Runs BreezySLAM continuously using your existing alex_lidar.py library.
Displays a live CLI map that updates with each scan.

Run this in a separate terminal alongside pi_sensor.py:
  Terminal 1: python3 slam_runner.py
  Terminal 2: python3 pi_sensor.py

Press Ctrl+C to stop and save the final map.
"""

import sys
import shutil
import time
import signal

from lidar.alex_lidar import (
    lidarConnect,
    lidarDisconnect,
    lidarStatus,
    performSingleScan
)
from breezy_slam import SLAMProcessor

# -----------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------
PORT     = "/dev/ttyUSB0"
BAUDRATE = 115200
MAP_SAVE_PATH = "/home/pi/final_map.pgm"  # adjust to your username


# -----------------------------------------------------------------------
# Terminal helpers (reuse style from your existing lidar_example_cli_plot)
# -----------------------------------------------------------------------
ESC = "\033["

def clear_lines(n):
    """Move cursor up n lines and clear downward."""
    sys.stdout.write(f"\r{ESC}{n}A{ESC}J")

def check_terminal_size(required_cols=82, required_rows=55):
    """Warn if terminal is too small."""
    cols, rows = shutil.get_terminal_size(fallback=(120, 60))
    if cols < required_cols or rows < required_rows:
        print(f"Warning: terminal too small ({cols}x{rows}). "
              f"Recommend at least {required_cols}x{required_rows}.")


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main():
    print("=" * 60)
    print("CG2111A SLAM Runner")
    print("=" * 60)
    check_terminal_size()

    # Initialise SLAM processor
    slam = SLAMProcessor()

    # Connect to LIDAR
    print(f"Connecting to LIDAR on {PORT}...")
    lidar = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
    status = lidarStatus(lidar, verbose=False)
    scan_mode = status['typical_scan_mode']
    print(f"LIDAR connected. Scan mode: {scan_mode}")
    print("Starting SLAM... Press Ctrl+C to stop and save map.\n")

    # Reserve terminal space for the map display
    # 50 lines for map + 1 for pose info + some margin
    display_lines = 52
    sys.stdout.write("\n" * display_lines)
    first_render = True

    # Handle Ctrl+C gracefully — save map before exit
    def on_exit(sig, frame):
        print("\n\nStopping SLAM...")
        slam.save_map(MAP_SAVE_PATH)
        lidarDisconnect(lidar)
        print("LIDAR disconnected.")
        sys.exit(0)

    signal.signal(signal.SIGINT, on_exit)

    # Main scan loop
    while True:
        try:
            # Get one full scan using your existing library function
            scan_data = performSingleScan(lidar, scan_mode)

            if scan_data is None:
                print("Warning: empty scan, retrying...")
                time.sleep(0.1)
                continue

            angles, distances, quality = scan_data

            # Update SLAM with new scan
            slam.update(angles, distances)

            # Render and display map
            map_str = slam.render_cli(display_size=80)

            if not first_render:
                clear_lines(display_lines)
            else:
                first_render = False

            sys.stdout.write(map_str + "\n")
            sys.stdout.flush()

        except Exception as e:
            print(f"Scan error: {e}")
            time.sleep(0.5)
            continue


if __name__ == '__main__':
    main()
