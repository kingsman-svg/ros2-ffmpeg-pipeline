#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/ffmpeg_pipeline_ws/install/local_setup.bash
exec "$@"
