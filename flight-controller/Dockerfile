# =============================================================================
# Flight Controller Dockerfile for ArduPilot SITL
# =============================================================================
# Provides pure ArduPilot SITL rover autopilot simulation without ROS dependencies.
# Implements hardware-authentic flight controller behavior for realistic testing.
# =============================================================================

FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies for ArduPilot SITL
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3 \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-matplotlib \
    python3-lxml \
    python3-pygame \
    libxml2-dev \
    libxslt-dev \
    wget \
    curl \
    build-essential \
    ccache \
    g++ \
    gawk \
    libtool \
    autoconf \
    netcat \
    && rm -rf /var/lib/apt/lists/*

# Create ardupilot user for secure operation
RUN useradd -m -s /bin/bash ardupilot && \
    echo "ardupilot:ardupilot" | chpasswd

# Switch to ardupilot user
USER ardupilot
WORKDIR /home/ardupilot

# Clone ArduPilot from official repository
# Use stable release branch for production reliability
RUN git clone https://github.com/ArduPilot/ardupilot.git && \
    cd ardupilot && \
    git submodule update --init --recursive

# Install Python dependencies for ArduPilot tools
# Note: ArduPilot no longer uses requirements.txt, install essential packages manually
RUN pip3 install --user pymavlink MAVProxy empy pexpect future

# Add ArduPilot tools to PATH for easy access
ENV PATH="/home/ardupilot/.local/bin:/home/ardupilot/ardupilot/Tools/autotest:$PATH"

# Create custom configuration directory structure
RUN mkdir -p /home/ardupilot/custom/{parameters,missions,scripts}

# Copy custom configurations from host
COPY --chown=ardupilot:ardupilot parameters/ /home/ardupilot/custom/parameters/
COPY --chown=ardupilot:ardupilot missions/ /home/ardupilot/custom/missions/
COPY --chown=ardupilot:ardupilot scripts/ /home/ardupilot/custom/scripts/

# Make all scripts executable
RUN chmod +x /home/ardupilot/custom/scripts/*.sh

# Expose MAVLink communication ports
# 14550: Primary MAVLink TCP/UDP port for flight computer communication
# 5760: Console/GCS connection port for debugging and monitoring
EXPOSE 14550/udp 5760

# Set default environment variables for ArduPilot SITL
ENV VEHICLE=APMrover2
ENV MODEL=rover
ENV LAT=37.7749
ENV LON=-122.4194
ENV SPEEDUP=1
ENV INSTANCE=0

# Use startup script as default command
CMD ["/home/ardupilot/custom/scripts/startup.sh"]