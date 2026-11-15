#ifndef VIDEO_PIPELINE_VIDEO_ENCODER_H_
#define VIDEO_PIPELINE_VIDEO_ENCODER_H_

#include <cstdint>
#include <string>

extern "C" {
#include <libavcodec/avcodec.h>
}

namespace video_pipeline
{

struct VideoEncoderConfig
{
  int width = 0;
  int height = 0;
  int frames_per_second = 30;
  std::int64_t bit_rate = 2000000;
  int gop_size = 30;
  int max_b_frames = 0;
  AVPixelFormat pixel_format = AV_PIX_FMT_YUV420P;
  bool global_header = false;
  std::string encoder_name;
  std::string tune;
};

class VideoEncoder
{
public:
  VideoEncoder();
  ~VideoEncoder();

  VideoEncoder(const VideoEncoder &) = delete;
  VideoEncoder & operator=(const VideoEncoder &) = delete;

  // 创建 H.264 编码器；encoder_name 为空时使用 FFmpeg 默认编码器。
  int open(const VideoEncoderConfig & config);

  // 输入帧的 PTS 使用 time_base() 的单位；传入 nullptr 表示输入结束。
  int send_frame(const AVFrame * frame);
  int receive_packet(AVPacket * packet);

  // parameters 由调用者分配和释放，可用于传递编码器参数。
  int copy_codec_parameters(AVCodecParameters * parameters) const;
  AVRational time_base() const;
  void close();

private:
  AVCodecContext * codec_context_;
};

}  // 结束 video_pipeline 命名空间

#endif
