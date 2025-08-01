#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/humble/setup.bash

# Set up proper environment for desktop session
export XDG_RUNTIME_DIR=/tmp/runtime-gazebo
export XDG_SESSION_CLASS=user
export XDG_SESSION_TYPE=x11
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Start a fresh Xvnc server
Xtigervnc :1 \
  -SecurityTypes None \
  -rfbport 5901 \
  -rfbauth /home/gazebo/.vnc/passwd \
  -desktop "ROS+Gazebo" \
  -geometry 1920x1080 \
  -depth 24 &

# Wait for X server to start
sleep 5

# Set display and desktop environment variables
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
export QT_QPA_PLATFORM=xcb
export QT_STYLE_OVERRIDE=kvantum
export GTK_THEME=Yaru

# Start D-Bus session bus for desktop communication
export $(dbus-launch)

# Start KWin window manager first (critical for window management)
kwin_x11 --replace &
sleep 2

# Start KDE Plasma desktop session
exec startplasma-x11
