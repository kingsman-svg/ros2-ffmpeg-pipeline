#include "video_pipeline/h264_decoder_strategy.h"

#include <cerrno>
#include <memory>

namespace video_pipeline
{

int SoftwareH264DecoderStrategy::open()
{
  const auto free_parameters = [](AVCodecParameters * parameters) {
      avcodec_parameters_free(&parameters);
    };
  std::unique_ptr<AVCodecParameters, decltype(free_parameters)> parameters(
    avcodec_parameters_alloc(), free_parameters);
  if (!parameters) {
    return AVERROR(ENOMEM);
  }

  // Annex B 码流中的 SPS 和 PPS 提供宽高等参数，打开时只需指定编码格式。
  parameters->codec_type = AVMEDIA_TYPE_VIDEO;
  parameters->codec_id = AV_CODEC_ID_H264;
  return decoder_.open(parameters.get());
}

int SoftwareH264DecoderStrategy::send_packet(const AVPacket * packet)
{
  return decoder_.send_packet(packet);
}

int SoftwareH264DecoderStrategy::receive_frame(AVFrame * frame)
{
  return decoder_.receive_frame(frame);
}

}  // 结束 video_pipeline 命名空间
