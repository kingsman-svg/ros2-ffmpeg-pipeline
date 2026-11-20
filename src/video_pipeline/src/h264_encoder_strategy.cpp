#include "video_pipeline/h264_encoder_strategy.h"

namespace video_pipeline
{

int SoftwareH264EncoderStrategy::open(const VideoEncoderConfig & config)
{
  // 低延迟调优由软件后端配置，节点只提供分辨率、帧率和码率等参数。
  VideoEncoderConfig software_config = config;
  software_config.encoder_name = "libx264";
  software_config.tune = "zerolatency";
  return encoder_.open(software_config);
}

int SoftwareH264EncoderStrategy::send_frame(const AVFrame * frame)
{
  return encoder_.send_frame(frame);
}

int SoftwareH264EncoderStrategy::receive_packet(AVPacket * packet)
{
  return encoder_.receive_packet(packet);
}

AVRational SoftwareH264EncoderStrategy::time_base() const
{
  return encoder_.time_base();
}

int SoftwareH264EncoderStrategy::set_bit_rate(std::int64_t bit_rate)
{
  return encoder_.set_bit_rate(bit_rate);
}

}  // 结束 video_pipeline 命名空间
