# =============================================================================
# Simulation Dockerfile for Pure Gazebo Physics
# =============================================================================
# Provides physics simulation and sensor modeling without flight computer logic.
# Uses existing optimized ROS2+Gazebo base image with additional control packages.
# =============================================================================

# Use existing custom ROS2+Gazebo image as base
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# Switch to root for package installation
USER root

# Install additional packages for ArduPilot Gazebo plugin and control
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-broadcaster \
    ros-humble-diff-drive-controller \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# NOTE: ArduPilot Gazebo plugin requires Gazebo Garden/newer (gz-cmake3)
# Current setup uses Gazebo Classic, so plugin installation is commented out
# Future versions could migrate to Gazebo Garden for ArduPilot plugin support
# RUN cd /tmp && \
#     git clone https://github.com/ArduPilot/ardupilot_gazebo.git && \
#     cd ardupilot_gazebo && \
#     mkdir build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
#     make -j4 && \
#     make install && \
#     cd / && rm -rf /tmp/ardupilot_gazebo

# Create gazebo user for consistency with GUI component
RUN useradd -m -s /bin/bash gazebo && \
    echo "gazebo:password12345!" | chpasswd

# Switch to gazebo user
USER gazebo
WORKDIR /home/gazebo

# Create workspace structure for simulation packages
RUN mkdir -p simulation_workspace/src

# Copy simulation source code and configurations
COPY --chown=gazebo:gazebo src/ simulation_workspace/src/
COPY --chown=gazebo:gazebo worlds/ simulation_workspace/worlds/
COPY --chown=gazebo:gazebo config/ simulation_workspace/config/
COPY --chown=gazebo:gazebo launch/ simulation_workspace/launch/
COPY --chown=gazebo:gazebo scripts/ simulation_workspace/scripts/

# Build ROS2 simulation packages
RUN cd simulation_workspace && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Set default environment variables for simulation
ENV ROS_DOMAIN_ID=0
ENV GAZEBO_MODEL_DATABASE_URI=""
ENV GAZEBO_MASTER_URI=http://localhost:11345
ENV LAT=37.7749
ENV LON=-122.4194

# Set Gazebo environment variables
# Note: ArduPilot plugin path removed since plugin not installed
ENV GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}
ENV GAZEBO_MODEL_PATH=/home/gazebo/simulation_workspace/worlds/models:${GAZEBO_MODEL_PATH}

# Set working directory
WORKDIR /home/gazebo/simulation_workspace