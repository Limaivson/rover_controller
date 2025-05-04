#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

pgrep pigpiod > /dev/null || pigpiod

exec "$@"
