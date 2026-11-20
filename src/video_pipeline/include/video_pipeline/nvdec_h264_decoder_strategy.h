#ifndef VIDEO_PIPELINE_NVDEC_H264_DECODER_STRATEGY_H_
#define VIDEO_PIPELINE_NVDEC_H264_DECODER_STRATEGY_H_

#include <memory>

#include "video_pipeline/ffmpeg_resources.h"
#include "video_pipeline/h264_decoder_strategy.h"

namespace video_pipeline
{

class NvdecH264DecoderStrategy : public H264DecoderStrategy
{
public:
  explicit NvdecH264DecoderStrategy(bool allow_software_fallback);

  int open() override;
  int send_packet(const AVPacket * packet) override;
  int receive_frame(AVFrame * frame) override;

private:
  struct CodecContextDeleter
  {
    void operator()(AVCodecContext * context) const;
  };

  static AVPixelFormat choose_pixel_format(
    AVCodecContext * context, const AVPixelFormat * formats);

  const bool allow_software_fallback_;
  std::unique_ptr<AVCodecContext, CodecContextDeleter> codec_context_;
  std::unique_ptr<AVFrame, FrameDeleter> decoded_frame_;
  bool hardware_frame_reported_ = false;
};

}  // 结束 video_pipeline 命名空间

#endif
