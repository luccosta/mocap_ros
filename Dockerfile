FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    libpcl-dev \
    libyaml-cpp-dev \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-launch* \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_WS=/ros2_ws
WORKDIR ${ROS_WS}

COPY src ./src

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install && \
    rm -r src

SHELL ["/bin/bash", "-c"]
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "icp_pose_estimator", "mocap_pose_estimator.launch.py"]