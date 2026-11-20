FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN sed -i 's|http://|https://|g' /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential pkg-config python3-colcon-common-extensions \
        ros-humble-ros2launch ros-humble-rosidl-default-generators ros-humble-sensor-msgs \
        ffmpeg libavformat-dev libavcodec-dev libavutil-dev libswscale-dev libswresample-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ffmpeg_pipeline_ws
COPY src/ src/
RUN source /opt/ros/humble/setup.bash \
    && colcon build --packages-up-to video_pipeline --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY scripts/ scripts/
COPY docker/entrypoint.sh /usr/local/bin/ffmpeg-pipeline-entrypoint.sh

ENTRYPOINT ["/bin/bash", "/usr/local/bin/ffmpeg-pipeline-entrypoint.sh"]
CMD ["/bin/bash", "/opt/ffmpeg_pipeline_ws/scripts/demo.sh"]
