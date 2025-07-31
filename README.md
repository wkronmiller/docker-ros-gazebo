# ROS2 Gazebo VNC Container

A containerized environment for robotics development featuring ROS2 Humble, Gazebo Classic simulation, and remote GUI access via VNC. This setup provides a complete desktop environment optimized for robotics development and simulation work.

## Quick Start

Get up and running in under 5 minutes:

### 1. Build the Container
```bash
git clone <your-repo-url>
cd gazebo
docker build -t ros2-gazebo-vnc .
```

### 2. Run the Container
```bash
# Basic usage
docker run -d \
  --name gazebo-desktop \
  -p 5901:5901 \
  --gpus all \
  ros2-gazebo-vnc

# With workspace mounting for development
docker run -d \
  --name gazebo-desktop \
  -p 5901:5901 \
  --gpus all \
  -v $(pwd)/workspace:/home/gazebo/workspace \
  ros2-gazebo-vnc

# Alternative: Using Docker Compose
docker compose up -d
```

### 3. Connect via VNC
- **VNC Client**: Connect to `localhost:5901`
- **VNC Password**: `1234`
- **Desktop**: Click the Gazebo icon on the desktop to launch the simulator

### 4. Login Credentials (if needed)
- The container automatically logs into the `gazebo` user, and the KDE screen locker is disabled to keep sessions active.
- **Username**: `gazebo`
- **Password**: `password12345!`

### 5. Verify Installation
Open a terminal (Konsole) in the VNC desktop and run:
```bash
# Check ROS2 installation
ros2 --version

# Launch Gazebo
source /opt/ros/humble/setup.bash
gazebo

# Test ROS2 + Gazebo integration
ros2 launch gazebo_ros empty_world.launch.py
```

That's it! You now have a complete ROS2 Gazebo development environment running in your browser.

## What This Package Is Used For

This Docker container is designed for:

- **Robotics Development**: Full ROS2 Humble environment with development tools
- **Robot Simulation**: Gazebo Classic for 3D robot simulation and testing
- **Remote Development**: VNC-based GUI access for headless server deployment
- **Educational Use**: Complete robotics environment for learning and teaching
- **CI/CD Pipelines**: Consistent environment for automated testing and simulation
- **Cross-Platform Development**: Unified robotics environment across different host systems

## How It Works

The container provides a complete Linux desktop environment accessible via VNC, with:

1. **CUDA-enabled Ubuntu 22.04** base system for GPU acceleration
2. **KDE Plasma Desktop** for a modern, professional GUI experience
3. **ROS2 Humble** with full desktop installation including RViz and rqt tools
4. **Gazebo Classic** robot simulator with ROS2 integration
5. **VNC Server** (TigerVNC) for remote desktop access on port 5901
6. **Non-root user security** - runs as `gazebo` user by default

### Connection Details
- **VNC Port**: 5901
- **VNC Password**: 1234 (configurable via build arg)
- **Desktop Resolution**: 1920x1080
- **User Account**: `gazebo` / `password12345!`

## File Structure and Purpose

### Core Container Files

#### `Dockerfile`
Multi-stage Docker build file with three stages:
- **Stage 0 (base)**: CUDA-enabled Ubuntu with KDE Plasma desktop and VNC server
- **Stage 1 (ros)**: ROS2 Humble installation with Gazebo and theming packages
- **Stage 2 (final)**: User configuration, VNC setup, and desktop customization

Key features:
- Non-root user setup for security
- Comprehensive theming for professional appearance
- Font configuration for crisp text rendering
- Desktop shortcuts and launchers

#### `start.sh`
Container startup script that:
- Sources ROS2 environment (`/opt/ros/humble/setup.bash`)
- Configures XDG runtime directories for desktop session
- Starts TigerVNC X server on display :1
- Initializes D-Bus session bus
- Launches KWin window manager
- Starts KDE Plasma desktop environment

Environment variables configured:
- `DISPLAY=:1` - X11 display for VNC
- `QT_X11_NO_MITSHM=1` - Fixes VNC compatibility issues
- `GAZEBO_MODEL_PATH` - Path to Gazebo model resources
- `QT_STYLE_OVERRIDE=kvantum` - Consistent Qt theming

### Desktop Configuration Files

#### `gazebo.desktop`
Desktop launcher shortcut for Gazebo simulator:
- **Type**: Application launcher
- **Exec**: `bash -c 'source /opt/ros/humble/setup.bash && gazebo'`
- **Category**: Development/Simulation
- Creates clickable desktop icon for easy Gazebo access

#### `fonts.conf`
Font rendering configuration for crisp text display:
- Enables font anti-aliasing for smooth text
- Configures hinting for better readability
- Sets LCD filter for optimal screen rendering
- Uses RGB sub-pixel rendering

#### `gtkrc-2.0`
GTK2 theme configuration ensuring visual consistency:
- **Theme**: Yaru (Ubuntu's modern theme)
- **Icons**: Yaru icon set
- **Font**: Ubuntu 10pt
- Ensures GTK2 applications match KDE Plasma appearance

### Documentation

#### `COMMIT_MSG`
Git commit message describing the container's evolution to non-root user operation, highlighting security improvements and performance optimizations.

## Docker Build Stages

### Stage 0: Base System (`base`)
```dockerfile
FROM nvidia/cuda:12.3.2-base-ubuntu22.04 as base
```

**Purpose**: Foundation layer with desktop environment and VNC capability

**Key Components**:
- NVIDIA CUDA 12.3.2 base for GPU acceleration
- KDE Plasma desktop environment (`kde-plasma-desktop`)
- Konsole terminal emulator
- KWin X11 window manager
- TigerVNC server components
- Virtual framebuffer X server (Xvfb)
- D-Bus session bus support
- Sudo for privilege management

**Size Impact**: ~2GB (desktop environment and CUDA base)

### Stage 1: ROS Installation (`ros`)
```dockerfile
FROM base AS ros
```

**Purpose**: Adds ROS2 Humble and Gazebo Classic with integration packages

**Key Components**:
- ROS2 Humble desktop meta-package (includes RViz, rqt tools)
- Gazebo Classic robot simulator
- ROS2-Gazebo bridge packages (`gazebo-ros`, `gazebo-ros-pkgs`)
- Python development tools (`rosdep2`, `colcon`)
- Comprehensive theming packages for professional GUI
- Font packages for text rendering

**Size Impact**: ~3GB additional (ROS2 ecosystem and Gazebo)

### Stage 2: Final Configuration (`final`)
```dockerfile
FROM ros AS final
```

**Purpose**: User setup, security configuration, and desktop customization

**Key Components**:
- Non-root `gazebo` user creation with sudo privileges
- VNC password configuration for both root and gazebo users
- Desktop environment setup (shortcuts, themes, fonts)
- Configuration file deployment
- Startup script installation
- Working directory and user context switching

**Size Impact**: Minimal (~50MB for configuration files)

## Usage

### Building the Container
```bash
docker build -t ros2-gazebo-vnc .
```

### Running the Container
```bash
docker run -d \
  --name gazebo-desktop \
  -p 5901:5901 \
  --gpus all \
  ros2-gazebo-vnc
```

### Accessing the Desktop
1. Connect to `localhost:5901` using any VNC client
2. Use password: `1234`
3. Launch Gazebo from the desktop shortcut or terminal

### Development Workflow
1. Mount your ROS2 workspace: `-v /path/to/workspace:/home/gazebo/workspace`
2. Use the integrated terminal (Konsole) for ROS2 commands
3. Access GUI tools like RViz and Gazebo through the desktop
4. Development files persist in mounted volumes

## Security Features

- **Non-root operation**: Container runs as `gazebo` user by default
- **Sudo access**: Available for administrative tasks when needed
- **VNC authentication**: Password-protected remote access
- **Isolated environment**: Containerized separation from host system
- **Secure defaults**: No SSH servers or unnecessary network services

## Performance Optimizations

- **GPU acceleration**: NVIDIA CUDA support for Gazebo rendering
- **Multi-stage builds**: Optimized layer caching and size
- **Build-time configuration**: User setup moved from runtime to build time
- **Efficient startup**: Streamlined desktop environment initialization
- **Font rendering**: Optimized for crisp text display over VNC

This container provides a professional, secure, and efficient environment for ROS2 robotics development with remote GUI access.