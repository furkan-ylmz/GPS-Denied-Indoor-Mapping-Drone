FROM osrf/ros:jazzy-desktop

# ══════════════════════════════════════════════════
# 1. SYSTEM TOOLS
# ══════════════════════════════════════════════════
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-venv \
    git \
    wget \
    curl \
    unzip \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1 \
    software-properties-common \
    lsb-release \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# ══════════════════════════════════════════════════
# 2. GAZEBO HARMONIC
# ══════════════════════════════════════════════════
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    && rm -rf /var/lib/apt/lists/*

# ══════════════════════════════════════════════════
# 3. ROS 2 — NAVIGATION, SLAM, TF, RVIZ
# ══════════════════════════════════════════════════
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

# ══════════════════════════════════════════════════
# 4. 3D SLAM — RTAB-Map + Point Cloud İşleme
# ══════════════════════════════════════════════════
RUN apt-get update && apt-get install -y \
    ros-jazzy-rtabmap \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-laser-geometry \
    ros-jazzy-depthimage-to-laserscan \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-rviz-plugins \
    ros-jazzy-octomap-ros \
    && rm -rf /var/lib/apt/lists/*

# ══════════════════════════════════════════════════
# 5. OCR & COMPUTER VISION (Kapı numarası tanıma)
# ══════════════════════════════════════════════════
RUN apt-get update && apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-vision-opencv \
    tesseract-ocr \
    tesseract-ocr-tur \
    libtesseract-dev \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Python OCR/AI kütüphaneleri
RUN pip3 install --break-system-packages \
    easyocr \
    pytesseract \
    Pillow \
    scipy \
    scikit-image

# ══════════════════════════════════════════════════
# 6. PX4 SITL BUILD DEPENDENCIES
# ══════════════════════════════════════════════════
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    gdb \
    astyle \
    libxml2-dev \
    libxml2-utils \
    libxslt1-dev \
    python3-jinja2 \
    python3-numpy \
    python3-empy \
    python3-toml \
    python3-packaging \
    python3-jsonschema \
    python3-future \
    python3-setuptools \
    python3-psutil \
    python3-cerberus \
    python3-pyros-genpymsg \
    default-jdk \
    && rm -rf /var/lib/apt/lists/*

# ══════════════════════════════════════════════════
# 7. PX4-AUTOPILOT
# ══════════════════════════════════════════════════
RUN git clone --recursive --depth 1 --branch main \
    https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
    && cd /opt/PX4-Autopilot \
    && DONT_RUN=1 make px4_sitl

# ══════════════════════════════════════════════════
# 8. MICRO XRCE-DDS AGENT (PX4 <-> ROS 2 bridge)
# ══════════════════════════════════════════════════
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/micro_xrce_agent \
    && cd /opt/micro_xrce_agent && mkdir build && cd build \
    && cmake .. && make && make install && ldconfig

# ══════════════════════════════════════════════════
# 9. PX4 MSGS (ROS 2 message tanımları)
# ══════════════════════════════════════════════════
RUN mkdir -p /root/drone_project/src && \
    git clone --branch main \
    https://github.com/PX4/px4_msgs.git /root/drone_project/src/px4_msgs

# ══════════════════════════════════════════════════
# 10. ENVIRONMENT SETUP
# ══════════════════════════════════════════════════
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/root/drone_project/models:/opt/PX4-Autopilot/Tools/simulation/gz/models" >> /root/.bashrc && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/PX4-Autopilot/build/px4_sitl_default/build_gz_plugins" >> /root/.bashrc && \
    echo "export PX4_ROOT=/opt/PX4-Autopilot" >> /root/.bashrc

WORKDIR /root/drone_project
