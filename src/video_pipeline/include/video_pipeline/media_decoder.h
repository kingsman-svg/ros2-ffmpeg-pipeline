#ifndef VIDEO_PIPELINE_MEDIA_DECODER_H_
#define VIDEO_PIPELINE_MEDIA_DECODER_H_

extern "C" {
#include <libavcodec/avcodec.h>
}

namespace video_pipeline
{

class MediaDecoder
{
public:
  MediaDecoder();
  ~MediaDecoder();

  MediaDecoder(const MediaDecoder &) = delete;
  MediaDecoder & operator=(const MediaDecoder &) = delete;

  // 支持音频和视频；初始化时复制参数，不依赖参数指针的生命周期。
  int open(const AVCodecParameters * parameters);

  // 传入 nullptr 表示输入结束，准备取出解码器中剩余的帧。
  // 保留 FFmpeg 的 EAGAIN 和 EOF 返回码，供调用者正确处理。
  int send_packet(const AVPacket * packet);
  int receive_frame(AVFrame * frame);
  void close();

private:
  AVCodecContext * codec_context_;
};

}  // 结束 video_pipeline 命名空间

#endif
