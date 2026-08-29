#!/usr/bin/env bash

set -e

echo "========================================="
echo "Setting up slam_ws"
echo "========================================="

WORKSPACE=$(cd "$(dirname "$0")" && pwd)

cd "$WORKSPACE"

###########################################################
# Check ROS
###########################################################

if [ "$ROS_DISTRO" != "jazzy" ]; then
    echo "This workspace requires ROS 2 Jazzy."
    echo "Current ROS_DISTRO = ${ROS_DISTRO:-<not sourced>}"
    
    exit 1
fi

echo "ROS_DISTRO = $ROS_DISTRO"

###########################################################
# Import repositories
###########################################################

if [ -f slam_ws.repos ]; then
    echo ""
    echo "Importing repositories..."

    mkdir -p src

    vcs import src < slam_ws.repos
fi

###########################################################
# Install dependencies
###########################################################

echo ""
echo "Installing rosdep dependencies..."

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y

###########################################################
# Build
###########################################################

echo ""
echo "Building workspace..."

colcon build \
    --cmake-clean-cache \
    --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3

###########################################################
# Source workspace
###########################################################

source install/setup.bash

echo ""
echo "========================================="
echo "Workspace ready!"
echo "========================================="