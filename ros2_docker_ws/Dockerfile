FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# Combine both installs into one RUN block
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ./src ./src

RUN source /opt/ros/humble/setup.bash && colcon build

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
