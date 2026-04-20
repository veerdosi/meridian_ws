#!/bin/bash
source /opt/ros/jazzy/setup.bash

# Start virtual display
Xvfb :99 -screen 0 1920x1080x24 &
export DISPLAY=:99

# Start VNC server (port 5900, no cursor, shared)
x11vnc -display :99 -passwd meridian -forever -shared -bg -noxdamage -quiet

exec "$@"
