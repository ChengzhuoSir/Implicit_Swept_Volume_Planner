#!/bin/bash
# ESV Planner - Environment initialization script
set -e

echo "=== ESV Planner Environment Setup ==="

# Check ROS
if [ -z "$ROS_DISTRO" ]; then
    echo "[WARN] ROS not sourced. Trying to source ROS Noetic..."
    if [ -f /opt/ros/noetic/setup.bash ]; then
        source /opt/ros/noetic/setup.bash
        echo "[OK] ROS Noetic sourced."
    else
        echo "[ERROR] ROS Noetic not found at /opt/ros/noetic/setup.bash"
        exit 1
    fi
else
    echo "[OK] ROS $ROS_DISTRO detected."
fi

# Check dependencies
echo "Checking dependencies..."
for pkg in roscpp geometry_msgs nav_msgs visualization_msgs std_msgs tf2 tf2_ros; do
    if rospack find "$pkg" > /dev/null 2>&1; then
        echo "  [OK] $pkg"
    else
        echo "  [MISSING] $pkg"
    fi
done

# Check Eigen3
if pkg-config --exists eigen3 2>/dev/null; then
    echo "  [OK] Eigen3 $(pkg-config --modversion eigen3)"
else
    echo "  [MISSING] Eigen3 - install with: sudo apt install libeigen3-dev"
fi

# Build
WORKSPACE_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
echo ""
echo "Workspace: $WORKSPACE_DIR"
echo "Building with catkin_make..."
cd "$WORKSPACE_DIR"
catkin_make -DCMAKE_CXX_STANDARD=14

echo ""
echo "=== Setup complete ==="
