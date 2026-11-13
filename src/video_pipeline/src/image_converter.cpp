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

int ImageConverter::convert(const AVFrame * input, AVPixelFormat output_format, AVFrame * output)
{
  if (input == nullptr || output == nullptr || input == output ||
    input->width <= 0 || input->height <= 0 || input->format < 0 ||
    input->data[0] == nullptr ||
    !sws_isSupportedInput(static_cast<AVPixelFormat>(input->format)) ||
    !sws_isSupportedOutput(output_format))
  {
    return AVERROR(EINVAL);
  }

  // 输入尺寸或格式变化时，FFmpeg 会重新创建转换上下文。
  scale_context_ = sws_getCachedContext(
    scale_context_, input->width, input->height,
    static_cast<AVPixelFormat>(input->format),
    input->width, input->height, output_format,
    SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (scale_context_ == nullptr) {
    return AVERROR(EINVAL);
  }

  av_frame_unref(output);
  output->width = input->width;
  output->height = input->height;
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
  if (result != input->height) {
    av_frame_unref(output);
    if (result < 0) {
      return result;
    }
    return AVERROR(EINVAL);
  }
  return 0;
}

}  // 结束 video_pipeline 命名空间
