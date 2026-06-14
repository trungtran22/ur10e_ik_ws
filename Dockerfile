FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash","-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository ppa:kisak/kisak-mesa -y
RUN apt-get update && apt-get dist-upgrade -y && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ur-description \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-tf2 \
    ros-humble-tf2-geometry-msgs \
    libeigen3-dev \
    python3-vcstool \
    python3-pip \
    cmake \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY src/ ./src/

RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc 
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
