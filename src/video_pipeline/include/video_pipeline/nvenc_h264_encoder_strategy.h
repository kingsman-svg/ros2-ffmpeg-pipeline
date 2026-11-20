#ifndef VIDEO_PIPELINE_NVENC_H264_ENCODER_STRATEGY_H_
#define VIDEO_PIPELINE_NVENC_H264_ENCODER_STRATEGY_H_

#include "video_pipeline/h264_encoder_strategy.h"

namespace video_pipeline
{

class NvencH264EncoderStrategy : public H264EncoderStrategy
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
