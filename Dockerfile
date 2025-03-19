FROM ros:humble-ros-core-jammy

# Set noninteractive mode to avoid prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=humble

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    build-essential \
    git \
    cmake \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    python3-pip \
    python3-rosdep \
    vim \
    wget && apt-get clean


RUN rosdep init && rosdep update
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN pip3 install -U colcon-common-extensions

RUN mkdir -p /home/rosuser/librealsense_ws/src
WORKDIR /home/rosuser/librealsense_ws/src

RUN git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    git checkout v2.53.1 && \
    cd /home/rosuser/librealsense_ws && \
    export MAKEFLAGS='-j2' && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install


RUN mkdir -p /home/rosuser/ros2_ws/src

ADD . /home/rosuser/ros2_ws/src/realsense-ros

WORKDIR /home/rosuser/ros2_ws

RUN source /opt/ros/humble/setup.bash && \
    source /home/rosuser/librealsense_ws/install/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys=librealsense2 && \
    colcon build --packages-select realsense2_camera_msgs realsense2_description --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install && \
    colcon build --packages-select realsense2_camera --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

RUN echo "source /home/rosuser/ros2_ws/install/local_setup.bash" >> ~/.bashrc
RUN echo "export MAKEFLAGS='-j2'" >> ~/.bashrc

# # Copy udev rules for RealSense devices after librealsense clone
RUN cp /home/rosuser/librealsense_ws/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules

# Default command to run a bash shell
CMD ["bash"]
