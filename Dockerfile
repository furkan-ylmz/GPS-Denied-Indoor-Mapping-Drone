FROM osrf/ros:jazzy-desktop

# ── System tools ──
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    curl \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1 \
    && rm -rf /var/lib/apt/lists/*

# ── Gazebo Harmonic (gz-sim for ROS 2 Jazzy) ──
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 Navigation & SLAM ──
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# ── Micro XRCE-DDS Agent (PX4 <-> ROS 2 bridge) ──
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/micro_xrce_agent \
    && cd /opt/micro_xrce_agent && mkdir build && cd build \
    && cmake .. && make && make install && ldconfig

# ── Environment setup ──
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/root/drone_project/models" >> /root/.bashrc

WORKDIR /root/drone_project
