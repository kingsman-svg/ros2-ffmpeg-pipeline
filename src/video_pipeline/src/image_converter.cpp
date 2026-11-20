#include "video_pipeline/image_converter.h"

#include <cerrno>

extern "C" {
#include <libavutil/error.h>
#include <libavutil/pixfmt.h>
}

namespace video_pipeline
{

ImageConverter::ImageConverter()
: scale_context_(nullptr)
{
}

ImageConverter::~ImageConverter()
{
  sws_freeContext(scale_context_);
}

int ImageConverter::convert(
  const AVFrame * input, AVPixelFormat output_format, AVFrame * output,
  int output_width, int output_height)
{
  if (input == nullptr || output == nullptr || input == output ||
    input->width <= 0 || input->height <= 0 || input->format < 0 ||
    input->data[0] == nullptr ||
    !sws_isSupportedInput(static_cast<AVPixelFormat>(input->format)) ||
    !sws_isSupportedOutput(output_format))
  {
    return AVERROR(EINVAL);
  }

  if (output_width == 0 && output_height == 0) {
    output_width = input->width;
    output_height = input->height;
  }
  if (output_width <= 0 || output_height <= 0) {
    return AVERROR(EINVAL);
  }

  // 一次转换同时处理像素格式和目标尺寸。
  scale_context_ = sws_getCachedContext(
    scale_context_, input->width, input->height,
    static_cast<AVPixelFormat>(input->format),
    output_width, output_height, output_format,
    SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (scale_context_ == nullptr) {
    return AVERROR(EINVAL);
  }

  av_frame_unref(output);
  output->width = output_width;
  output->height = output_height;
  output->format = output_format;

  int result = av_frame_get_buffer(output, 32);
  if (result < 0) {
    return result;
  }

  result = av_frame_copy_props(output, input);
  if (result < 0) {
    av_frame_unref(output);
    return result;
  }

  result = sws_scale(
    scale_context_, input->data, input->linesize, 0, input->height,
    output->data, output->linesize);
  if (result != output_height) {
    av_frame_unref(output);
    if (result < 0) {
      return result;
    }
    return AVERROR(EINVAL);
  }
  return 0;
}

}  // 结束 video_pipeline 命名空间
