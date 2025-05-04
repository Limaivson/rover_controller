FROM ros:humble-ros-base


RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-pigpio \
    pigpio \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /ros2_ws
COPY ros2_ws /ros2_ws


RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "rover_controller", "rover_controller"]
