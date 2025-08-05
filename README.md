# Multi-Component AI Rover Docker Images

This repository builds Docker images for a multi-component AI rover system with ArduPilot SITL simulator support.

## Architecture

The repository produces six Docker images organized in two systems:

### Layered Base Images
- **`docker-ros-base`** - Base ROS2 Humble environment
- **`docker-ros-gazebo`** - Gazebo simulation and navigation stack
- **`docker-ros-gazebo-gui`** - GUI desktop environment with VNC

### Multi-Component Images  
- **`rover-flight-controller`** - Pure ArduPilot SITL rover autopilot
- **`rover-flight-computer`** - ROS2 navigation stack with MAVROS bridge
- **`rover-simulation`** - Gazebo physics simulation with sensor modeling

## Quick Start

### Build All Images
```bash
# Build everything (base images + multi-component)
make all

# Build only base layered images
make base gazebo gui

# Build only multi-component images  
make multicomponent
```

### Build Individual Images
```bash
# Base system
make base          # ROS2 Humble foundation
make gazebo        # Gazebo + Navigation2
make gui           # Desktop GUI with VNC

# Multi-component system
make flight-controller  # ArduPilot SITL
make flight-computer   # ROS2 + MAVROS
make simulation        # Gazebo physics
```

### Publish Images
```bash
# Publish all images with current git tag
make publish

# Publish and tag as latest
make publish-latest

# Publish specific sets
make publish-base           # Base images only
make publish-multicomponent # Multi-component only
```

## Image Details

### Base Images

#### **docker-ros-base** (`ghcr.io/wkronmiller/docker-ros-base`)
**Size:** ~3.6GB | **Base:** NVIDIA CUDA 12.3.2 Ubuntu 22.04

**Contains:**
- Ubuntu 22.04 LTS with NVIDIA CUDA 12.3.2 support for GPU acceleration
- ROS2 Humble desktop meta-package (RViz, rqt, navigation tools)
- Python 3.10 with rosdep2 and colcon build system
- Essential development tools and dependencies

**Use Cases:**
- Foundation for any ROS2 Humble application
- Base layer for custom robotics containers
- Development environment for ROS2 projects
- CUDA-enabled applications requiring ROS2

#### **docker-ros-gazebo** (`ghcr.io/wkronmiller/docker-ros-gazebo`)
**Size:** ~4.2GB | **Base:** docker-ros-base

**Contains:**
- Gazebo Classic simulation environment with full physics engine
- ROS2-Gazebo integration packages (gazebo-ros, gazebo-ros-pkgs, gazebo-plugins)
- Navigation2 complete stack for autonomous navigation
- SLAM Toolbox for simultaneous localization and mapping
- Robot localization package for sensor fusion
- ROS2 Control framework and controllers (diff_drive_controller, joint_state_broadcaster)
- Teleop keyboard control for manual robot operation

**Use Cases:**
- Robot simulation and testing environment
- Navigation algorithm development and testing
- Multi-robot system simulation
- Foundation for custom Gazebo simulations
- Autonomous navigation research and development

#### **docker-ros-gazebo-gui** (`ghcr.io/wkronmiller/docker-ros-gazebo-gui`)
**Size:** ~4.8GB | **Base:** docker-ros-gazebo

**Contains:**
- KDE Plasma desktop environment optimized for robotics workflows
- TigerVNC server (port 5901) with password authentication
- X11 forwarding and display management (DISPLAY=:1)
- Custom font rendering and GTK theming for professional appearance
- Pre-configured Gazebo desktop launcher
- Non-root 'gazebo' user with sudo privileges for secure operation
- Automatic desktop login and screen lock disabled for continuous operation

**Use Cases:**
- Remote robotics development and visualization
- Gazebo GUI access through VNC clients
- RViz and rqt tool visualization
- Educational robotics environments
- Collaborative robot development sessions

### Multi-Component Images

#### **rover-flight-controller** (`ghcr.io/wkronmiller/rover-flight-controller`)
**Size:** ~3.7GB | **Base:** Ubuntu 22.04

**Contains:**
- ArduPilot SITL (Software In The Loop) complete rover autopilot system
- Python 3.10 with essential scientific packages (opencv, matplotlib, pygame)
- PyMAVLink, MAVProxy, and ArduPilot communication tools
- Custom rover parameters and mission files in `/home/ardupilot/custom/`
- ArduPilot source code with all submodules for customization
- MAVLink communication ports (14550/UDP primary, 5760 console)
- Dedicated 'ardupilot' user for secure operation

**Use Cases:**
- Hardware-authentic flight controller simulation
- ArduPilot algorithm development and testing
- MAVLink protocol testing and validation
- Rover autopilot behavior analysis
- Mission planning and execution testing
- Integration testing with external flight computers

**Environment Variables:**
- `VEHICLE=APMrover2` - ArduPilot vehicle type
- `MODEL=rover` - Simulation model configuration
- `LAT/LON` - GPS home position coordinates
- `SPEEDUP=1` - Simulation time multiplier

#### **rover-flight-computer** (`ghcr.io/wkronmiller/rover-flight-computer`)
**Size:** ~4.3GB | **Base:** docker-ros-gazebo

**Contains:**
- Complete ROS2 Humble environment with all Gazebo and Navigation2 packages
- MAVROS bridge for ArduPilot communication with full message support
- MAVROS extras for advanced autopilot integration features
- GeographicLib datasets for GPS coordinate transformations
- Custom ROS2 workspace with flight computer packages
- Pre-built packages for immediate deployment
- Dedicated 'gazebo' user matching other components

**Use Cases:**
- High-level rover navigation and mission planning
- ArduPilot-ROS2 bridge operations
- Autonomous navigation with obstacle avoidance
- Sensor data processing and fusion
- Mission execution and monitoring
- Integration with external planning systems

**Key Features:**
- MAVROS FCU URL configured for flight-controller communication
- ROS2 domain ID 0 for multi-robot coordination
- Symlinked installation for rapid development iteration

#### **rover-simulation** (`ghcr.io/wkronmiller/rover-simulation`)
**Size:** ~4.2GB | **Base:** docker-ros-gazebo

**Contains:**
- Complete Gazebo Classic physics simulation environment
- ROS2 Control framework with rover-specific controllers
- Robot state publisher and joint state management
- Additional control packages for comprehensive robot simulation
- Custom simulation workspace with worlds and configurations
- Pre-built ROS2 packages for immediate use
- Dedicated 'gazebo' user for consistency

**Use Cases:**
- Physics-accurate rover simulation
- Sensor modeling and testing (GPS, IMU, cameras, LiDAR)
- Environmental interaction simulation
- Multi-robot scenario testing
- Hardware-in-the-loop simulation preparation
- Algorithm validation in realistic physics

**Key Features:**
- ROS2 domain ID 0 for seamless multi-component communication
- Configurable world models and sensor configurations
- Empty Gazebo model database for custom model integration
- GPS coordinates matching flight controller for consistency

**Note:** ArduPilot Gazebo plugin integration is prepared but commented out due to compatibility with Gazebo Classic. Future versions may migrate to Gazebo Garden for full plugin support.

## System Architecture & Communication

### Multi-Component Communication Flow
```
┌─────────────────────┐    MAVLink/UDP     ┌─────────────────────┐
│  flight-controller  │◄──────────────────►│  flight-computer    │
│  (ArduPilot SITL)   │     Port 14550     │  (ROS2 + MAVROS)    │
└─────────────────────┘                    └─────────────────────┘
                                                     │
                                                     │ ROS2 Topics
                                                     │ Domain ID 0
                                                     ▼
┌─────────────────────┐    Gazebo Physics   ┌─────────────────────┐
│   GUI (Optional)    │◄──────────────────►│     simulation      │
│  (VNC Visualization)│     Port 11345     │  (Gazebo Physics)   │
└─────────────────────┘                    └─────────────────────┘
```

### Port Mappings
- **14550/UDP**: MAVLink communication (flight-controller ↔ flight-computer)
- **5760/TCP**: ArduPilot console/GCS connection
- **5901/TCP**: VNC server for GUI access
- **11345/TCP**: Gazebo master URI for physics simulation

### Network Configuration
All components use **ROS2 Domain ID 0** for seamless topic communication between flight-computer and simulation containers.

## File Structure

```
docker-ros-gazebo/
├── base/
│   └── Dockerfile              # ROS2 base image
├── gazebo/  
│   └── Dockerfile              # Gazebo + navigation
├── gui/
│   ├── Dockerfile              # GUI desktop
│   ├── entrypoint.sh
│   └── fonts.conf
├── flight-controller/
│   ├── Dockerfile              # ArduPilot SITL
│   ├── parameters/rover.parm
│   ├── missions/default_mission.txt
│   └── scripts/startup.sh
├── flight-computer/
│   ├── Dockerfile              # ROS2 + MAVROS
│   ├── src/mavros_bridge/      # Custom MAVROS bridge
│   ├── config/mavros_config.yaml
│   ├── launch/flight_computer.launch.py
│   └── scripts/entrypoint.sh
├── simulation/
│   ├── Dockerfile              # Gazebo physics
│   ├── worlds/rover_world.world
│   ├── config/controllers.yaml
│   ├── launch/simulation.launch.py
│   └── scripts/entrypoint.sh
├── Makefile                    # Build system
└── MULTI_COMPONENT.md          # Architecture specification
```

## Available Make Targets

### Build Targets
- `make all` - Build all images (default)
- `make base` - Build ROS2 base image
- `make gazebo` - Build Gazebo image  
- `make gui` - Build GUI image
- `make flight-controller` - Build ArduPilot SITL image
- `make flight-computer` - Build ROS2 + MAVROS image
- `make simulation` - Build Gazebo physics image
- `make multicomponent` - Build all multi-component images

### Publishing Targets
- `make publish` - Push all images to registry
- `make publish-latest` - Push and tag as latest
- `make publish-base` - Push base images only
- `make publish-multicomponent` - Push multi-component images only

### Testing Targets
- `make test-flight-controller` - Test ArduPilot SITL image
- `make test-containers` - Test all images

### Utility Targets
- `make clean` - Clean build artifacts
- `make clean-all` - Remove all built images
- `make print-tag` - Show current git tag
- `make print-image-names` - Show all image names and tags
- `make help` - Show all available targets

## Usage Examples

### Pull Pre-built Images
```bash
# Pull published images
docker pull ghcr.io/wkronmiller/rover-flight-controller:latest
docker pull ghcr.io/wkronmiller/rover-flight-computer:latest  
docker pull ghcr.io/wkronmiller/rover-simulation:latest

# Or use the GUI image for visualization
docker pull ghcr.io/wkronmiller/docker-ros-gazebo-gui:latest
```

### Docker Compose Example
```yaml
# docker-compose.yml - Complete rover simulation system
version: '3.8'
services:
  flight-controller:
    image: ghcr.io/wkronmiller/rover-flight-controller:latest
    ports:
      - "14550:14550/udp"  # MAVLink communication
      - "5760:5760"        # Console access
    environment:
      - VEHICLE=APMrover2
      - MODEL=rover
      - LAT=37.7749
      - LON=-122.4194

  flight-computer:
    image: ghcr.io/wkronmiller/rover-flight-computer:latest
    depends_on:
      - flight-controller
    environment:
      - ROS_DOMAIN_ID=0
      - MAVROS_FCU_URL=udp://flight-controller:14550@

  simulation:
    image: ghcr.io/wkronmiller/rover-simulation:latest
    environment:
      - ROS_DOMAIN_ID=0
      - GAZEBO_MASTER_URI=http://localhost:11345
    ports:
      - "11345:11345"      # Gazebo master

  # Optional: GUI for visualization
  gui:
    image: ghcr.io/wkronmiller/docker-ros-gazebo-gui:latest
    ports:
      - "5901:5901"        # VNC access
    environment:
      - DISPLAY=:1
```

### Individual Container Usage
```bash
# Run ArduPilot SITL flight controller
docker run -d --name flight-controller \
  -p 14550:14550/udp -p 5760:5760 \
  ghcr.io/wkronmiller/rover-flight-controller:latest

# Run ROS2 flight computer with MAVROS
docker run -d --name flight-computer \
  --link flight-controller \
  -e MAVROS_FCU_URL=udp://flight-controller:14550@ \
  ghcr.io/wkronmiller/rover-flight-computer:latest

# Run Gazebo simulation
docker run -d --name simulation \
  -p 11345:11345 \
  ghcr.io/wkronmiller/rover-simulation:latest

# Access GUI via VNC (password: 1234)
docker run -d --name gui \
  -p 5901:5901 \
  ghcr.io/wkronmiller/docker-ros-gazebo-gui:latest
```

## Architecture Benefits

### Hardware Authenticity
- **Separate containers** mirror real flight controller + companion computer deployment
- **Real MAVLink protocols** between components
- **Independent failure domains** and resource allocation

### Development Efficiency
- **Component isolation** enables focused debugging
- **Independent builds** and deployment
- **Clear separation** of concerns between autopilot, navigation, and simulation

### Deployment Ready
- **Same images** work on real hardware
- **Realistic communication patterns**
- **Production-ready** container architecture

## See Also

- `MULTI_COMPONENT.md` - Complete multi-component architecture specification
- `Makefile` - Complete build system documentation
