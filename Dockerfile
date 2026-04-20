FROM arm64v8/ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-ur-description \
    libgl1 \
    libgl1-mesa-dri \
    libegl1 \
    libglu1-mesa \
    mesa-utils \
    mesa-vulkan-drivers \
    xvfb \
    x11-utils \
    x11vnc \
    openbox \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install mujoco scipy numpy --break-system-packages

RUN python3 -m pip install anthropic matplotlib --break-system-packages

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# VNC password (change if desired)
RUN mkdir -p /root/.vnc && x11vnc -storepasswd meridian /root/.vnc/passwd

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 8765

WORKDIR /root/meridian_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
