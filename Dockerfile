FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-launch-testing-ros \
    python3-matplotlib \
    curl ca-certificates \
    inotify-tools \
    && rm -rf /var/lib/apt/lists/*

# install just
RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh \
    | bash -s -- --to /usr/local/bin

WORKDIR /ws
COPY src/ src/
COPY justfile justfile
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

RUN echo 'source /ws/install/setup.bash' >> /root/.bashrc
CMD ["bash"]
