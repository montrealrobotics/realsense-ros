# Use Ubuntu 20.04 as base image
FROM ubuntu:20.04

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Define argument for ROS version (default to Noetic)
ARG ROS_VERSION=noetic

# Install necessary tools and add the ROS repository
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && if [ "$ROS_VERSION" = "noetic" ]; then \
        sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'; \
    else \
        sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'; \
    fi

# Install ROS based on the provided ROS_VERSION
RUN if [ "$ROS_VERSION" = "noetic" ]; then \
        apt-get update && apt-get install -y \
        ros-noetic-desktop-full \
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
        wget && apt-get clean; \
    else \
        apt-get update && apt-get install -y \
        ros-galactic-desktop \
	ros-galactic-ament-cmake \
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
        wget && apt-get clean; \
    fi

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up the ROS environment based on ROS version
RUN if [ "$ROS_VERSION" = "noetic" ]; then \
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
        /bin/bash -c "source /opt/ros/noetic/setup.bash"; \
    else \
        echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc && \
        /bin/bash -c "source /opt/ros/galactic/setup.bash"; \
    fi

# Install Intel RealSense SDK from source (version 2.53.1)
WORKDIR /tmp
RUN git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    git checkout v2.53.1 && \
    mkdir build && cd build && \
    cmake ../ -DCMAKE_BUILD_TYPE=Release && \
    make -j4 && sudo make install

# Copy udev rules for RealSense devices and reload the udev rules
RUN cp /tmp/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules

# Set LD_LIBRARY_PATH for runtime
RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc


# Set up workspace and build RealSense ROS packages based on ROS version
RUN if [ "$ROS_VERSION" = "noetic" ]; then \
        apt-get update && apt-get install -y python3-catkin-tools && \
        mkdir -p /home/rosuser/catkin_ws/src && \
        cd /home/rosuser/catkin_ws/src && \
        git clone https://github.com/IntelRealSense/realsense-ros.git && \
        cd realsense-ros && \
        git checkout ros1-legacy && \
        cd /home/rosuser/catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash && \
    	rosdep install --from-paths src --ignore-src -r -y --skip-keys=librealsense2 && \
    	catkin_make"; \
    else \
        pip3 install -U colcon-common-extensions && \
        mkdir -p /home/rosuser/ros2_ws/src && \
        cd /home/rosuser/ros2_ws/src && \
        git clone https://github.com/IntelRealSense/realsense-ros.git && \
        cd realsense-ros && \
        git checkout ros2-legacy && \
	apt-get update && apt-get install -y \
    	ros-galactic-xacro \
    	ros-galactic-diagnostic-updater && \
	cd /home/rosuser/ros2_ws && \
	/bin/bash -c "source /opt/ros/galactic/setup.bash && \
        rosdep install --from-paths src --ignore-src -r -y --skip-keys=librealsense2 && \
        cd /home/rosuser/ros2_ws && \
	colcon build --packages-select realsense2_camera_msgs --cmake-args -DCMAKE_PREFIX_PATH=/usr/local -DCMAKE_BUILD_TYPE=Release && \
	colcon build --packages-select realsense2_camera --cmake-args -DCMAKE_PREFIX_PATH=/usr/local -DCMAKE_BUILD_TYPE=Release"; \
    fi

# Set environment variables for the selected ROS version
RUN if [ "$ROS_VERSION" = "noetic" ]; then \
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
        echo "source /home/rosuser/catkin_ws/devel/setup.bash" >> ~/.bashrc; \
    else \
        echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc && \
        echo "source /home/rosuser/ros2_ws/install/local_setup.bash" >> ~/.bashrc; \
    fi

RUN pip install pyrealsense2

# Default command to run a bash shell
CMD ["bash"]

