#!/bin/bash
# =============================================================================
# Flight Computer Entrypoint Script
# =============================================================================
# Initializes ROS2 environment and starts navigation stack with MAVROS bridge
# Waits for flight controller availability before starting services
# =============================================================================

set -e

echo "============================================================================="
echo "Flight Computer Initialization"
echo "============================================================================="
echo "ROS Domain ID: ${ROS_DOMAIN_ID:-0}"
echo "MAVROS FCU URL: ${MAVROS_FCU_URL:-udp://flight-controller:14550@}"
echo "============================================================================="

# Source ROS2 environments
source /opt/ros/humble/setup.bash
cd /home/gazebo/Documents/amee-rover/flight_computer_workspace
source install/setup.bash

# Wait for flight controller to be available
echo "Waiting for flight controller connection..."
FLIGHT_CONTROLLER_HOST=$(echo ${MAVROS_FCU_URL} | sed -n 's/.*:\/\/\([^:]*\):.*/\1/p')
FLIGHT_CONTROLLER_PORT=$(echo ${MAVROS_FCU_URL} | sed -n 's/.*:\([0-9]*\).*/\1/p')

echo "Checking connection to ${FLIGHT_CONTROLLER_HOST}:${FLIGHT_CONTROLLER_PORT}..."
until nc -z ${FLIGHT_CONTROLLER_HOST} ${FLIGHT_CONTROLLER_PORT}; do
  echo "Flight controller not ready, waiting..."
  sleep 2
done

echo "Flight controller connection established!"
sleep 2

# Start flight computer navigation stack
echo "Starting flight computer navigation and MAVROS bridge..."
exec ros2 launch /home/gazebo/Documents/amee-rover/flight_computer_workspace/launch/flight_computer.launch.py