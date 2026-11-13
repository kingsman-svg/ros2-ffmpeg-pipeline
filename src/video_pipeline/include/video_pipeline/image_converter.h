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

  // 输出与输入等尺寸、指定像素格式的图像；两个帧均由调用者持有。
  // 每次调用会先清空 output 中原有的数据。
  int convert(const AVFrame * input, AVPixelFormat output_format, AVFrame * output);

private:
  SwsContext * scale_context_;
};

}  // 结束 video_pipeline 命名空间

#endif
