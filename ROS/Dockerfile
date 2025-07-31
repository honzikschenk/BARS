# syntax=docker/dockerfile:1
FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    nano \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    numpy \
    scipy \
    matplotlib \
    PyYAML \
    pyserial \
    setuptools

# Set up workspace
WORKDIR /workspace
ENV ROS_WS=/workspace

# Copy ROS2 workspace
COPY . /workspace/src/

# Initialize rosdep if not already done
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi

# Update rosdep
RUN rosdep update

# Install dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Setup ROS environment\n\
source /opt/ros/humble/setup.bash\n\
source /workspace/install/setup.bash\n\
\n\
# Execute command\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]