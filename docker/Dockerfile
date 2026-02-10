# -----------------------------
# Lightweight ROS Humble Image
# -----------------------------
FROM ros:humble-ros-base

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# -----------------------------
# Install only essential tools
# -----------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3-pip \
        python3-colcon-common-extensions \
        nano \
        curl \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# -----------------------------
# Create ROS workspace
# -----------------------------
WORKDIR /roar_ws
RUN mkdir -p src

# -----------------------------
# Automatically source ROS
# -----------------------------
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# -----------------------------
# Set default entrypoint
# -----------------------------
CMD ["bash"]
COPY Dockerfile /roar_ws/Dockerfile
