#!/usr/bin/env bash
# One-command build for the F1TENTH simulator.
#   1. Installs the f110_gym Python package and compiles the C++ simulation core.
#   2. Builds the ROS 2 bridge with colcon (skipped when ROS 2 is not installed).
set -e
cd "$(dirname "$0")"

echo "[1/2] Installing f110_gym (compiles the C++ simulation core)..."
pip3 install -e .

ROS_SETUP="$(ls /opt/ros/*/setup.bash 2>/dev/null | sort | tail -n 1)"
if [ -z "$ROS_SETUP" ]; then
    echo "[2/2] ROS 2 not found - skipping the ROS 2 bridge build."
    echo
    echo "Done. Try the gym API directly:"
    echo "  cd examples && python3 waypoint_follow.py"
    exit 0
fi

echo "[2/2] Building the ROS 2 bridge (using $ROS_SETUP)..."
source "$ROS_SETUP"
colcon build --symlink-install --base-paths f1tenth_gym_ros

echo
echo "Done. Run the simulator with:"
echo "  source install/setup.bash"
echo "  ros2 launch f1tenth_gym_ros gym_bridge_launch.py"
