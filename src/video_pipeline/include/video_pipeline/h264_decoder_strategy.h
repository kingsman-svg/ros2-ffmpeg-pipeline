#ifndef VIDEO_PIPELINE_H264_DECODER_STRATEGY_H_
#define VIDEO_PIPELINE_H264_DECODER_STRATEGY_H_

#include "video_pipeline/media_decoder.h"

namespace video_pipeline
{

class H264DecoderStrategy
{
public:
  virtual ~H264DecoderStrategy() = default;

  virtual int open() = 0;
  // 接收 Annex B 编码包。传入 nullptr 时排出剩余解码帧。
  virtual int send_packet(const AVPacket * packet) = 0;
  // 返回可在 CPU 上读取的图像帧，并保留 FFmpeg 的 EAGAIN 和 EOF 返回码。
  virtual int receive_frame(AVFrame * frame) = 0;
};

class SoftwareH264DecoderStrategy : public H264DecoderStrategy
{
public:
  int open() override;
  int send_packet(const AVPacket * packet) override;
  int receive_frame(AVFrame * frame) override;

private:
  MediaDecoder decoder_;
};

}  // 结束 video_pipeline 命名空间

#endif
