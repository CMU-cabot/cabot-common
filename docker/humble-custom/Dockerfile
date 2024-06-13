ARG FROM_IMAGE

FROM $FROM_IMAGE as base

ENV ROS_DISTRO=humble

# downgrade libopencv-dev for ROS2
RUN if [ "$TARGETARCH" = "linux/arm64" ]; then \
        apt update && apt install -q -y --no-install-recommends --allow-downgrades \
        libopencv-dev=4.5.4+dfsg-9ubuntu4 \
        && \
        apt clean && \
        rm -rf /var/lib/apt/lists/* \
        ; \
    fi

# add gazebo repository for arm
RUN if [ "$TARGETARCH" = "linux/arm64" ]; then \
        apt update && apt install -q -y --no-install-recommends \
        software-properties-common \
        && \
        apt clean && \
        rm -rf /var/lib/apt/lists/* && \
	add-apt-repository ppa:openrobotics/gazebo11-non-amd64 \
        ; \
    fi

RUN apt update && \
    apt install -y --no-install-recommends \
    libapr1-dev \
    libaprutil1-dev \
    libasio-dev \
    libpcap-dev \
    python3-pip \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-cyclonedds \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-gazebo-dev \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-osrf-testing-tools-cpp \
    ros-$ROS_DISTRO-test-msgs \
    && \
    rm -rf /var/lib/apt/lists/*

FROM base as cache

ENV CUSTOM_DIR=/opt/custom_ws

RUN mkdir -p $CUSTOM_DIR/src
COPY . $CUSTOM_DIR/src

WORKDIR $CUSTOM_DIR

RUN apt update && rosdep update && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y --skip-keys "xacro librealsense2 message_runtime rslidar_input rslidar_msg"

RUN . /opt/ros/humble/setup.sh && \
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release  # --executor sequential

FROM base as build

ENV CUSTOM_DIR=/opt/custom_ws

RUN mkdir -p $CUSTOM_DIR/install
COPY --from=cache $CUSTOM_DIR/install $CUSTOM_DIR/install
RUN sed -i "s:/ros/\$ROS_DISTRO/:/custom_ws/install/:" /ros_entrypoint.sh

