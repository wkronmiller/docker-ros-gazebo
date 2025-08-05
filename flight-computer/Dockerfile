# =============================================================================
# Flight Computer Dockerfile for ROS2 Navigation + MAVROS
# =============================================================================
# Provides high-level navigation, perception, and MAVROS bridge functionality.
# Uses existing optimized ROS2+Gazebo base image with MAVROS additions.
# =============================================================================

# Use existing custom ROS2+Gazebo image as base
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# Switch to root for package installation
USER root

# Install MAVROS and additional navigation dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs \
    ros-humble-geographic-msgs \
    python3-pip \
    wget \
    netcat \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets for MAVROS GPS coordinate transformations
# Required for proper GPS/local coordinate system conversions
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Create gazebo user for consistency with GUI component
RUN useradd -m -s /bin/bash gazebo && \
    echo "gazebo:password12345!" | chpasswd

# Switch to gazebo user
USER gazebo
WORKDIR /home/gazebo

# Create workspace structure for flight computer packages
RUN mkdir -p flight_computer_workspace/src

# Copy flight computer source code and configurations
COPY --chown=gazebo:gazebo src/ flight_computer_workspace/src/
COPY --chown=gazebo:gazebo config/ flight_computer_workspace/config/
COPY --chown=gazebo:gazebo launch/ flight_computer_workspace/launch/
COPY --chown=gazebo:gazebo scripts/ flight_computer_workspace/scripts/

# Build ROS2 flight computer packages
# Source ROS2 environment and build with symlink install for development
RUN cd flight_computer_workspace && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Set default environment variables for flight computer operation
ENV ROS_DOMAIN_ID=0
ENV MAVROS_FCU_URL=udp://flight-controller:14550@

# Set working directory
WORKDIR /home/gazebo/flight_computer_workspace