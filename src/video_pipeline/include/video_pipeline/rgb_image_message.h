#ifndef VIDEO_PIPELINE_RGB_IMAGE_MESSAGE_H_
#define VIDEO_PIPELINE_RGB_IMAGE_MESSAGE_H_

#include <cstddef>
#include <cstdint>

extern "C" {
#include <libavutil/imgutils.h>
}

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "video_pipeline/ffmpeg_resources.h"

namespace video_pipeline
{

inline sensor_msgs::msg::Image make_rgb_image(
  const AVFrame & rgb_frame, const std_msgs::msg::Header & header)
{
  const int width = rgb_frame.width;
  const int height = rgb_frame.height;
  const int image_size = av_image_get_buffer_size(AV_PIX_FMT_RGB24, width, height, 1);
  if (image_size < 0) {
    throw ffmpeg_error("计算 RGB 图像大小失败", image_size);
  }

  sensor_msgs::msg::Image image;
  image.header = header;
  image.width = static_cast<std::uint32_t>(width);
  image.height = static_cast<std::uint32_t>(height);
  image.encoding = sensor_msgs::image_encodings::RGB8;
  image.is_bigendian = 0;
  image.step = image.width * 3;
  image.data.resize(static_cast<std::size_t>(image_size));

  // 按 FFmpeg 帧的行跨度读取，以 1 字节对齐写入，去掉行末填充。
  const std::uint8_t * planes[4] = {rgb_frame.data[0], nullptr, nullptr, nullptr};
  const int copied = av_image_copy_to_buffer(
    image.data.data(), image_size, planes, rgb_frame.linesize, AV_PIX_FMT_RGB24,
    width, height, 1);
  if (copied < 0) {
    throw ffmpeg_error("复制 RGB 图像失败", copied);
  }
  return image;
}

}  // 结束 video_pipeline 命名空间

#endif
