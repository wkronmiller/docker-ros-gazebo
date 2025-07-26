# =============================================================================
# Multi-stage Dockerfile for ROS2 Humble + Gazebo Classic with VNC GUI access
# =============================================================================
# This Dockerfile creates a containerized environment for robotics development
# with ROS2, Gazebo simulation, and remote GUI access via VNC.
#
# Architecture:
# - Stage 0 (base): CUDA-enabled Ubuntu with KDE Plasma desktop and VNC server
# - Stage 1 (ros): ROS2 Humble installation with Gazebo Classic integration  
# - Stage 2 (final): User setup, configuration files, and VNC customization
# =============================================================================

# -----------------------------------------------------------------------------
# Stage 0: Base system with CUDA support and desktop environment
# -----------------------------------------------------------------------------
FROM nvidia/cuda:12.3.2-base-ubuntu22.04 as base

# Prevent interactive prompts during package installation
# Essential for automated Docker builds
ENV DEBIAN_FRONTEND=noninteractive

# Install complete desktop environment and VNC server components
# - kde-plasma-desktop: Full KDE Plasma desktop environment for GUI applications
# - konsole: Terminal emulator integrated with KDE
# - plasma-workspace: Core Plasma shell and workspace components
# - kwin-x11: KDE window manager for X11 (essential for window management)
# - xvfb: Virtual framebuffer X server for headless operation
# - tigervnc-*: VNC server implementation for remote desktop access
# - tightvncserver: Additional VNC tools (provides vncpasswd utility)
# - dbus-x11: D-Bus session bus for desktop application communication
# - sudo: Allow non-root users to execute privileged commands
RUN apt-get update && apt-get install -y --no-install-recommends \
      kde-plasma-desktop konsole plasma-workspace kwin-x11 \
      xvfb \
      tigervnc-standalone-server tigervnc-common tightvncserver \
      dbus-x11 sudo \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Stage 1: ROS2 Humble installation with Gazebo Classic integration
# -----------------------------------------------------------------------------
FROM base AS ros

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
         > /etc/apt/sources.list.d/ros2.list

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

# Install Gazebo Classic simulation environment and ROS2 integration packages
# - gazebo: Gazebo Classic robot simulation software
# - ros-*-gazebo-ros: Core ROS2-Gazebo bridge and communication layer
# - ros-*-gazebo-ros-pkgs: Additional ROS2-Gazebo integration packages
# - ros-*-gazebo-plugins: Sensor and actuator plugins for Gazebo simulations
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      gazebo \
      ros-${ROS_DISTRO}-gazebo-ros \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs \
      ros-${ROS_DISTRO}-gazebo-plugins \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-controller-manager \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-gazebo-ros2-control \
      ros-${ROS_DISTRO}-joint-state-broadcaster \
      ros-${ROS_DISTRO}-diff-drive-controller \
      ros-${ROS_DISTRO}-robot-localization \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-slam-toolbox \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      && rm -rf /var/lib/apt/lists/*

# Install comprehensive theming and font packages for professional GUI appearance
# Font packages:
# - fontconfig: Font configuration and management system
# - fonts-dejavu, fonts-liberation: High-quality open source fonts
# GTK/Qt theming packages:
# - gtk2-engines-*: GTK2 theme engines for consistent widget appearance
# - gnome-themes-extra, adwaita-icon-theme: Modern theme components
# - ubuntu-mono, yaru-theme-*: Ubuntu's default theme and icon set
# - qt5-style-*: Qt5 style plugins for theme consistency across toolkits
# - gsettings-desktop-schemas: Configuration schemas for desktop settings
RUN apt-get update && apt-get install -y --no-install-recommends \
      fontconfig fonts-dejavu fonts-liberation \
      gtk2-engines-murrine gtk2-engines-pixbuf \
      gnome-themes-extra adwaita-icon-theme \
      ubuntu-mono yaru-theme-gtk yaru-theme-icon \
      qt5-style-plugins qt5-gtk-platformtheme \
      qt5-style-kvantum qt5-style-kvantum-themes \
      gsettings-desktop-schemas \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Stage 2: Final configuration with VNC setup and user management
# -----------------------------------------------------------------------------
FROM ros AS final

# Define VNC password as build argument (can be overridden at build time)
# Default password is "1234" for easy access during development
ARG VNC_PASSWORD=1234

# Create VNC configuration directory and set password for root user
# - Creates .vnc directory in root's home
# - Uses vncpasswd to create encrypted password file
# - Sets restrictive permissions (600) for security
RUN mkdir -p /root/.vnc \
 && printf "${VNC_PASSWORD}\n${VNC_PASSWORD}\n\n" | vncpasswd -f > /root/.vnc/passwd \
 && chmod 600 /root/.vnc/passwd

# Configure environment variables for GUI applications
# - DISPLAY=:1: Set X11 display to VNC server display number
# Commented variables for potential future use:
# - QT_X11_NO_MITSHM: Disable MIT-SHM extension (fixes some VNC issues)
# - GAZEBO_MODEL_PATH: Custom path for Gazebo model resources
ENV DISPLAY=:1
#ENV QT_X11_NO_MITSHM=1
#ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models

# Set up desktop environment with Gazebo launcher shortcut
# - Creates Desktop directory for user shortcuts
# - Copies Gazebo application launcher to desktop
# - Makes launcher executable so it appears properly in desktop environment
RUN mkdir -p /root/Desktop
COPY gazebo.desktop /root/Desktop/gazebo.desktop
RUN chmod +x /root/Desktop/gazebo.desktop

# Install GTK2 theme configuration for consistent widget appearance
# - Copies custom GTK2 configuration to user's home directory
# - Ensures GTK2 applications match the KDE Plasma theme
COPY gtkrc-2.0 /root/.gtkrc-2.0

# Configure font rendering with anti-aliasing for crisp text display
# - Creates fontconfig directory structure
# - Copies custom font configuration for improved text rendering
# - Essential for professional appearance in GUI applications
RUN mkdir -p /root/.config/fontconfig
COPY fonts.conf /root/.config/fontconfig/fonts.conf

# Install custom startup script that configures VNC and launches desktop
# - Copies startup script to system binary directory
# - Makes script executable so it can run as container entrypoint
# - Script handles VNC server startup and desktop environment initialization
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Create non-root user account for secure operation
# - useradd: Creates 'gazebo' user with home directory and bash shell
# - chpasswd: Sets password to 'password12345!' for login access
# - usermod: Adds user to sudo group for administrative privileges
# This allows VNC sessions to run as non-root user for better security
RUN useradd -m -s /bin/bash gazebo \
    && echo "gazebo:password12345!" | chpasswd \
    && usermod -aG sudo gazebo

# Set up gazebo user environment (operations that require root)
# - Create VNC directory and password file for gazebo user
# - Set up desktop directory with Gazebo shortcut
# - Copy configuration files to gazebo user's home
# - Set proper ownership and permissions for all gazebo user files
RUN mkdir -p /home/gazebo/.vnc \
    && printf "1234\n1234\n\n" | vncpasswd -f > /home/gazebo/.vnc/passwd \
    && chmod 600 /home/gazebo/.vnc/passwd \
    && mkdir -p /home/gazebo/Desktop \
    && cp /root/Desktop/gazebo.desktop /home/gazebo/Desktop/ \
    && cp /root/.gtkrc-2.0 /home/gazebo/ \
    && mkdir -p /home/gazebo/.config/fontconfig \
    && cp /root/.config/fontconfig/fonts.conf /home/gazebo/.config/fontconfig/ \
    && chown -R gazebo:gazebo /home/gazebo

# Configure automatic KDE login
RUN mkdir -p /etc/sddm.conf.d && \
    echo "[Autologin]" > /etc/sddm.conf.d/autologin.conf && \
    echo "User=gazebo" >> /etc/sddm.conf.d/autologin.conf && \
    echo "Session=plasma" >> /etc/sddm.conf.d/autologin.conf

# Remove sudo access for gazebo user and disable root access
RUN usermod -d /home/gazebo -s /bin/bash gazebo && \
    deluser gazebo sudo 2>/dev/null || true && \
    passwd -l root && \
    sed -i 's/^gazebo:.*$/gazebo:x:1000:1000:gazebo:\/home\/gazebo:\/bin\/bash/' /etc/passwd

# Switch to gazebo user for all subsequent operations
# This ensures the container runs as non-root by default
USER gazebo

# Set up working directory as gazebo user's home
WORKDIR /home/gazebo

# Set startup script as container entrypoint
# - Script now runs directly as gazebo user without privilege switching
# - Handles VNC server startup and desktop environment initialization
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

