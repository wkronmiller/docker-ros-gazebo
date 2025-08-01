# =============================================================================
# Base Dockerfile for ROS2 Humble
# =============================================================================
# Provides a minimal ROS2 environment built on top of CUDA-enabled Ubuntu.
# Additional Dockerfiles layer on Gazebo and GUI support.
# =============================================================================

FROM nvidia/cuda:12.3.2-base-ubuntu22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set ROS2 distribution version as environment variable for consistency
ENV ROS_DISTRO=humble

# Add the official ROS2 apt repository to package sources
# - gnupg2, lsb-release, curl: Required for repository key verification
# - Downloads and adds ROS2 GPG key for package authentication
# - Adds ROS2 repository URL to apt sources for the current Ubuntu release
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      gnupg2 lsb-release curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         | apt-key add - \
    && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 desktop meta-package and build tools
# - ros-humble-desktop: Complete ROS2 installation with GUI tools (RViz, rqt, etc.)
# - python3-rosdep2: Dependency management tool for ROS packages
# - python3-colcon-common-extensions: Build system for ROS2 workspaces
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-desktop \
      python3-rosdep2 python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep database for dependency resolution
# - rosdep init: Creates system-wide rosdep configuration (may fail if already exists)
# - rosdep update: Downloads package dependency database
RUN rosdep init || true && \
    rosdep update
