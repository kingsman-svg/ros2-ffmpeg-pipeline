#!/usr/bin/env bash
set -eo pipefail

demo_dir=$(mktemp -d)
trap 'rm -rf -- "$demo_dir"' EXIT

# 生成固定帧数的样片，让演示在没有摄像头或视频文件时也能运行。
ffmpeg -nostdin -hide_banner -loglevel error \
  -f lavfi -i 'testsrc2=size=320x240:rate=10' \
  -frames:v 30 -c:v mpeg4 "$demo_dir/input.mp4"

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
ros2 launch video_pipeline video_pipeline.launch.py \
  "input_path:=$demo_dir/input.mp4" fps:=10 \
  output_width:=160 output_height:=120 \
  encoder_backend:=software decoder_backend:=software
