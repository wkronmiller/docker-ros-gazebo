#!/bin/bash
# =============================================================================
# Simulation Entrypoint Script
# =============================================================================
# Initializes Gazebo physics simulation with rover model and sensor plugins
# Coordinates GPS reference frame with flight controller for proper alignment
# =============================================================================

set -e

echo "============================================================================="
echo "Gazebo Simulation Initialization"
echo "============================================================================="
echo "ROS Domain ID: ${ROS_DOMAIN_ID:-0}"
echo "Gazebo Master URI: ${GAZEBO_MASTER_URI:-http://localhost:11345}"
echo "GPS Reference: ${LAT:-37.7749}, ${LON:-122.4194}"
echo "============================================================================="

# Source ROS2 environments
source /opt/ros/humble/setup.bash
cd /home/gazebo/Documents/amee-rover/simulation_workspace
source install/setup.bash

# Set Gazebo environment variables for ArduPilot plugin
export GAZEBO_PLUGIN_PATH=/usr/local/lib/gazebo:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=/home/gazebo/Documents/amee-rover/simulation_workspace/worlds/models:$GAZEBO_MODEL_PATH

# Wait a moment for system initialization
sleep 2

# Start Gazebo simulation with rover model
echo "Starting Gazebo physics simulation..."
exec ros2 launch /home/gazebo/Documents/amee-rover/simulation_workspace/launch/simulation.launch.py \
    world_file:=/home/gazebo/Documents/amee-rover/simulation_workspace/worlds/rover_world.world \
    gps_lat:=${LAT:-37.7749} \
    gps_lon:=${LON:-122.4194}