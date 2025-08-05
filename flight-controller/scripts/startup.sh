#!/bin/bash
# =============================================================================
# ArduPilot SITL Startup Script
# =============================================================================
# Starts ArduPilot SITL rover simulation with proper MAVLink configuration
# for multi-component container architecture communication
# =============================================================================

set -e

echo "============================================================================="
echo "Starting ArduPilot SITL Rover Simulation"
echo "============================================================================="
echo "Vehicle: ${VEHICLE:-APMrover2}"
echo "Model: ${MODEL:-rover}"
echo "Home Location: ${LAT:-37.7749}, ${LON:-122.4194}"
echo "Simulation Speed: ${SPEEDUP:-1}x"
echo "Instance: ${INSTANCE:-0}"
echo "============================================================================="

# Change to ArduPilot directory
cd /home/ardupilot/ardupilot

# Wait a moment for system initialization
sleep 2

# Start ArduPilot SITL with optimized parameters for container communication
# Key configurations:
# - --no-mavproxy: Disable MAVProxy to avoid conflicts
# - --speedup: Control simulation speed (1 = real-time)
# - --defaults: Load custom parameter file
# - --home: Set GPS home position (must match simulation container)
# - --out: MAVLink output configuration for container networking
exec python3 Tools/autotest/sim_vehicle.py \
    -v "${VEHICLE:-APMrover2}" \
    --model "${MODEL:-rover}" \
    --no-mavproxy \
    --speedup "${SPEEDUP:-1}" \
    --defaults "/home/ardupilot/custom/parameters/rover.parm" \
    --home "${LAT:-37.7749},${LON:-122.4194},50,0" \
    --out tcpin:0.0.0.0:14550 \
    --instance "${INSTANCE:-0}"