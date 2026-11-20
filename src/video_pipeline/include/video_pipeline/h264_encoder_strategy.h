#ifndef VIDEO_PIPELINE_H264_ENCODER_STRATEGY_H_
#define VIDEO_PIPELINE_H264_ENCODER_STRATEGY_H_

#include "video_pipeline/video_encoder.h"

namespace video_pipeline
{

class H264EncoderStrategy
{
public:
  virtual ~H264EncoderStrategy() = default;

  virtual int open(const VideoEncoderConfig & config) = 0;
  // 输入帧的 PTS 使用 time_base() 的单位。传入 nullptr 时排出剩余编码包。
  virtual int send_frame(const AVFrame * frame) = 0;
  // 取出 Annex B 编码包，调用者根据 FFmpeg 返回码决定何时继续送帧。
  virtual int receive_packet(AVPacket * packet) = 0;
  virtual AVRational time_base() const = 0;
  virtual int set_bit_rate(std::int64_t bit_rate) = 0;
};

class SoftwareH264EncoderStrategy : public H264EncoderStrategy
{
public:
  int open(const VideoEncoderConfig & config) override;
  int send_frame(const AVFrame * frame) override;
  int receive_packet(AVPacket * packet) override;
  AVRational time_base() const override;
  int set_bit_rate(std::int64_t bit_rate) override;

private:
  VideoEncoder encoder_;
};

}  // 结束 video_pipeline 命名空间

#endif
