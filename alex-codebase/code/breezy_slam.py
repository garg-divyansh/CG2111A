"""
breezy_slam.py
==============
BreezySLAM integration for CG2111A Final Project.

Provides:
  - SLAMProcessor class: wraps BreezySLAM for use with your existing
    alex_lidar.py scan data format
  - CLI map renderer: renders the SLAM map to terminal
  - PGM map saver: saves final map as an image file

Usage:
  from breezy_slam import SLAMProcessor
  slam = SLAMProcessor()
  slam.update(angles, distances)   # same format as your existing scan data
  print(slam.render_cli())         # print map to terminal
  slam.save_map('final_map.pgm')   # save at end of run
"""

import numpy as np
from PIL import Image
import time
import sys

# -----------------------------------------------------------------------
# BreezySLAM imports with clear error message if not installed
# -----------------------------------------------------------------------
try:
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import RPLidarA1
except ImportError:
    print("ERROR: BreezySLAM not installed.")
    print("Run: cd BreezySLAM/python && sudo python3 setup.py install")
    sys.exit(1)


# -----------------------------------------------------------------------
# Constants — tuned for your 3x3m arena
# -----------------------------------------------------------------------

# Map size in mm (4000mm = 4m, gives margin around 3x3m arena)
MAP_SIZE_MM     = 4000

# Map resolution: pixels per metre equivalent
# 100 pixels / 4m = 0.04m = 4cm per pixel — good for your arena
MAP_SIZE_PIXELS = 800

# RPLidar A1 scan parameters
LIDAR_MIN_MM    = 120    # minimum valid range in mm
LIDAR_MAX_MM    = 3500   # maximum valid range in mm (slightly beyond arena)

# Number of angle samples BreezySLAM expects
# RPLidar A1 produces ~360-1000 points per scan
# We resample everything to 360 (1 per degree) for consistency
SCAN_SIZE       = 360


# -----------------------------------------------------------------------
# SLAMProcessor
# -----------------------------------------------------------------------

class SLAMProcessor:
    """
    Wraps BreezySLAM's RMHC_SLAM algorithm.

    RMHC = Random Mutation Hill Climbing
    This is a particle-filter based SLAM that works without odometry.
    It estimates robot position purely from successive LIDAR scans.
    """

    def __init__(self):
        # Initialise the RPLidar A1 sensor model
        # This tells BreezySLAM the physical characteristics of your sensor
        self.sensor = RPLidarA1()

        # Initialise RMHC_SLAM
        # - sensor:         physical sensor model
        # - map_size:       map width/height in pixels
        # - map_size_mm:    real-world size the map represents
        # - random_seed:    for reproducibility
        self.slam = RMHC_SLAM(
            self.sensor,
            MAP_SIZE_PIXELS,
            MAP_SIZE_MM,
            random_seed=42
        )

        # Map storage: flat array of bytes, MAP_SIZE_PIXELS^2 elements
        # 0 = occupied (wall), 255 = free space, 127 = unknown
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

        # Track robot pose for display
        self.robot_x_mm   = MAP_SIZE_MM / 2
        self.robot_y_mm   = MAP_SIZE_MM / 2
        self.robot_theta  = 0.0

        # Track whether we have a valid first scan
        self.scan_count = 0

    def _resample_scan(self, angles, distances_mm):
        """
        Resample raw LIDAR scan to exactly SCAN_SIZE points (one per degree).

        BreezySLAM expects a fixed-size array of distances indexed by angle.
        Your RPLidar produces variable numbers of points per scan.
        This function bins the raw points into 360 degree buckets.

        Args:
            angles:       list/array of angles in degrees (0-360)
            distances_mm: list/array of distances in mm

        Returns:
            list of SCAN_SIZE distances in mm, one per degree
        """
        # Initialise output array with zeros (zero = invalid/no reading)
        resampled = [0] * SCAN_SIZE

        # For each raw point, put it in the correct degree bucket
        for angle, dist in zip(angles, distances_mm):
            # Normalise angle to 0-359
            idx = int(angle) % SCAN_SIZE

            # Filter invalid readings
            if dist < LIDAR_MIN_MM or dist > LIDAR_MAX_MM:
                continue

            # If multiple points fall in same bucket, keep the closest
            # (conservative: walls are better detected by nearest point)
            if resampled[idx] == 0 or dist < resampled[idx]:
                resampled[idx] = int(dist)

        return resampled

    def update(self, angles, distances_mm):
        """
        Process a new LIDAR scan and update the SLAM map.

        Args:
            angles:       list of angles in degrees — same format as
                         your existing performSingleScan() output
            distances_mm: list of distances in mm — same format as
                         your existing performSingleScan() output

        Call this once per scan in your main loop.
        """
        # Resample to fixed size
        scan = self._resample_scan(angles, distances_mm)

        # Update SLAM — no odometry so we pass None
        # BreezySLAM will estimate motion from scan matching alone
        self.slam.update(scan, pose_change=None)

        # Get updated robot pose
        x_mm, y_mm, theta_deg = self.slam.getpos()
        self.robot_x_mm  = x_mm
        self.robot_y_mm  = y_mm
        self.robot_theta = theta_deg

        # Update map bytes
        self.slam.getmap(self.mapbytes)

        self.scan_count += 1

    def get_pose(self):
        """
        Returns current estimated robot pose.
        Returns: (x_mm, y_mm, theta_degrees)
        """
        return self.robot_x_mm, self.robot_y_mm, self.robot_theta

    def render_cli(self, display_size=80):
        """
        Render the current SLAM map as a CLI string for terminal display.

        Downsamples the full MAP_SIZE_PIXELS map to display_size columns
        for terminal display. Shows robot position as 'R'.

        Args:
            display_size: number of columns in terminal output

        Returns:
            string ready to print to terminal
        """
        CHARS = " ░▒▓█"

        # Downsample factor
        step = MAP_SIZE_PIXELS // display_size
        if step < 1:
            step = 1

        display_height = MAP_SIZE_PIXELS // step

        # Robot position in map pixels
        robot_px = int((self.robot_x_mm / MAP_SIZE_MM) * MAP_SIZE_PIXELS)
        robot_py = int((self.robot_y_mm / MAP_SIZE_MM) * MAP_SIZE_PIXELS)

        lines = []
        for row in range(display_height - 1, -1, -1):
            line = []
            for col in range(display_size):
                # Map display pixel back to map array index
                map_row = row * step
                map_col = col * step
                idx = map_row * MAP_SIZE_PIXELS + map_col

                # Check bounds
                if idx >= len(self.mapbytes):
                    line.append(' ')
                    continue

                # Check if this is near robot position
                if (abs(map_col - robot_px) < step * 2 and
                        abs(map_row - robot_py) < step * 2):
                    line.append("\033[31mR\033[0m")  # red R
                    continue

                # Convert map value to display character
                # mapbytes: 255=free(white), 0=occupied(black), 127=unknown
                val = self.mapbytes[idx]

                if val > 200:
                    # Free space — show as empty
                    line.append(' ')
                elif val < 50:
                    # Occupied — show as solid
                    line.append('\033[32m█\033[0m')  # green walls
                else:
                    # Unknown
                    line.append('░')

            lines.append("".join(line))

        # Add pose info at bottom
        lines.append(
            f"\033[36mPos: x={self.robot_x_mm/1000:.2f}m "
            f"y={self.robot_y_mm/1000:.2f}m "
            f"θ={self.robot_theta:.1f}°  "
            f"Scans: {self.scan_count}\033[0m"
        )

        return "\n".join(lines)

    def save_map(self, filepath='final_map.pgm'):
        """
        Save the current map as a PGM greyscale image.

        The saved image can be opened in any image viewer.
        Walls appear dark, free space appears light.

        Args:
            filepath: path to save the .pgm file
        """
        # Convert flat bytearray to numpy array then to PIL image
        map_array = np.frombuffer(self.mapbytes, dtype=np.uint8)
        map_array = map_array.reshape((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))

        # Flip vertically so north is up
        map_array = np.flipud(map_array)

        img = Image.fromarray(map_array, mode='L')
        img.save(filepath)
        print(f"Map saved to: {filepath}")
        print(f"Map size: {MAP_SIZE_PIXELS}x{MAP_SIZE_PIXELS} pixels")
        print(f"Real-world coverage: {MAP_SIZE_MM/1000}m x {MAP_SIZE_MM/1000}m")
