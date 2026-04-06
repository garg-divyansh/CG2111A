#!/bin/bash
# One-time setup on Raspberry Pi (Ubuntu 20.04)
set -e

echo "=== Installing ROS Noetic ==="
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop python3-rosdep \
  python3-rosinstall python3-rosinstall-generator \
  python3-wstool build-essential python3-catkin-tools

sudo rosdep init 2>/dev/null || true
rosdep update

echo "=== Creating catkin workspace ==="
SLAM_DIR="$(dirname "$(realpath "$0")")/catkin_ws"
mkdir -p "$SLAM_DIR/src"
cd "$SLAM_DIR/src"

echo "=== Cloning rplidar_ros and hector_slam ==="
[ -d rplidar_ros ] || git clone https://github.com/Slamtec/rplidar_ros.git
[ -d hector_slam ] || git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git

echo "=== Installing dependencies ==="
cd "$SLAM_DIR"
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y

echo "=== Building ==="
catkin_make

echo ""
echo "=== Done! Now add this to your ~/.bashrc ==="
echo "source $SLAM_DIR/devel/setup.bash"
