#!/usr/bin/env bash
# One-command build for the F1TENTH simulator.
#   1. Installs the f110_gym Python package and compiles the C++ simulation core.
#   2. Builds the ROS 2 bridge with colcon (skipped when ROS 2 is not installed).
set -e
cd "$(dirname "$0")"

if ! python3 -m pip --version >/dev/null 2>&1; then
    echo "ERROR: pip is not available. Install it first:  sudo apt install python3-pip"
    exit 1
fi

# PEP 668 (Ubuntu >= 23.04): installing outside a venv needs --break-system-packages
PIP_FLAGS=""
if [ -z "$VIRTUAL_ENV" ] && \
   [ -f "$(python3 -c 'import sysconfig,os; print(os.path.join(sysconfig.get_path("stdlib"), "EXTERNALLY-MANAGED"))')" ]; then
    PIP_FLAGS="--user --break-system-packages"
fi

echo "[1/2] Installing f110_gym (compiles the C++ simulation core)..."
python3 -m pip install $PIP_FLAGS -e .

# prefer the distro already sourced in this shell, then jazzy (the distro this
# repo targets), otherwise the newest installed one
if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    ROS_SETUP=/opt/ros/jazzy/setup.bash
else
    ROS_SETUP="$(ls /opt/ros/*/setup.bash 2>/dev/null | sort | tail -n 1)"
fi
if [ -z "$ROS_SETUP" ]; then
    echo "[2/2] ROS 2 not found - skipping the ROS 2 bridge build."
    echo
    echo "Done. Try the gym API directly:"
    echo "  cd examples && python3 waypoint_follow.py"
    exit 0
fi

echo "[2/2] Building the ROS 2 bridge (using $ROS_SETUP)..."
source "$ROS_SETUP"

# install the bridge's ROS dependencies declared in package.xml
if command -v rosdep >/dev/null 2>&1; then
    rosdep install -i --from-path f1tenth_gym_ros --rosdistro "$ROS_DISTRO" -y || {
        echo "WARNING: rosdep could not install dependencies."
        echo "         If rosdep has never been set up, run: sudo rosdep init && rosdep update"
        echo "         Or install them manually (see README, Requirements)."
    }
else
    echo "NOTE: rosdep not found - install the bridge dependencies manually (see README, Requirements)."
fi

colcon build --symlink-install --base-paths f1tenth_gym_ros

echo
echo "Done. Run the simulator with:"
echo "  source install/setup.bash   # or setup.zsh"
echo "  ros2 launch f1tenth_gym_ros gym_bridge_launch.py"
