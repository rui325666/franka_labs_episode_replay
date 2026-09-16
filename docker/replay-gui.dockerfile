# Reuse the camera's ROS/CycloneDDS/cv_bridge environment.
FROM registry.localhost/labs/zed-camera:latest
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-numpy python3-pil python3-matplotlib python3-pyqt5 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /tmp
CMD ["/bin/bash"]
