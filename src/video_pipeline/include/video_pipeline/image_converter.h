#ifndef VIDEO_PIPELINE_IMAGE_CONVERTER_H_
#define VIDEO_PIPELINE_IMAGE_CONVERTER_H_

extern "C" {
#include <libavutil/frame.h>
#include <libswscale/swscale.h>
}

namespace video_pipeline
{

class ImageConverter
{
public:
  ImageConverter();
  ~ImageConverter();

  ImageConverter(const ImageConverter &) = delete;
  ImageConverter & operator=(const ImageConverter &) = delete;

  // 目标宽高均为 0 时采用输入尺寸。输入帧和输出帧由调用者持有。
  // 分配输出缓冲前会清空 output 中原有的数据。
  int convert(
    const AVFrame * input, AVPixelFormat output_format, AVFrame * output,
    int output_width = 0, int output_height = 0);

private:
  SwsContext * scale_context_;
};

}  // 结束 video_pipeline 命名空间

#endif
