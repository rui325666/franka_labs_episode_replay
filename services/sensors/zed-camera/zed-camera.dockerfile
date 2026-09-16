####################################################################################################
# Stage: base
FROM ros:jazzy-ros-base AS base

ARG ZED_OPEN_CAPTURE_COMMIT=93739154903a66c112ee52c3c6fbfd83b7218617

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    libusb-1.0-0-dev \
    python3-colcon-common-extensions \
    ros-jazzy-cv-bridge \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-sensor-msgs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Build only the CPU/UVC video module. Pinning the commit makes container builds reproducible.
RUN git clone https://github.com/stereolabs/zed-open-capture.git /tmp/zed-open-capture && \
    git -C /tmp/zed-open-capture checkout "${ZED_OPEN_CAPTURE_COMMIT}" && \
    cmake \
      -S /tmp/zed-open-capture \
      -B /tmp/zed-open-capture/build \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_SENSORS=OFF \
      -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/zed-open-capture/build --parallel "$(nproc)" && \
    cmake --install /tmp/zed-open-capture/build && \
    ldconfig && \
    rm -rf /tmp/zed-open-capture

WORKDIR /workspace

COPY src/zed_open_capture_ros/package.xml ros2_ws/src/zed_open_capture_ros/package.xml
COPY src/zed_open_capture_ros/CMakeLists.txt ros2_ws/src/zed_open_capture_ros/CMakeLists.txt
COPY src/zed_open_capture_ros/src ros2_ws/src/zed_open_capture_ros/src

RUN source /opt/ros/jazzy/setup.bash && \
    cd /workspace/ros2_ws && \
    colcon build \
      --packages-select zed_open_capture_ros \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY entrypoint.sh entrypoint.sh
COPY config_zed_camera.yml config_zed_camera.yml

ARG BUILD_CREATED
ARG BUILD_TARGET
ARG BUILD_VERSION
ARG GIT_COMMIT
LABEL \
    de.franka.image.build-target=$BUILD_TARGET \
    de.franka.image.created=$BUILD_CREATED \
    de.franka.image.git-commit=$GIT_COMMIT \
    de.franka.image.title="ZED Open Capture Camera" \
    de.franka.image.version=$BUILD_VERSION \
    de.franka.service.name="zed-camera"
ENV BUILD_CREATED=$BUILD_CREATED \
    BUILD_TARGET=$BUILD_TARGET \
    BUILD_VERSION=$BUILD_VERSION \
    GIT_COMMIT=$GIT_COMMIT

CMD ["/bin/bash", "./entrypoint.sh"]

####################################################################################################
# Stage: dev
FROM base AS dev

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    iputils-ping \
    vim-tiny \
    v4l-utils \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

####################################################################################################
# Stage: prod
FROM base AS prod
