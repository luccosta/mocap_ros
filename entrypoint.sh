#!/bin/bash
set -e

# Fonte o ROS 2 e o workspace
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Executa o comando passado
exec "$@"
