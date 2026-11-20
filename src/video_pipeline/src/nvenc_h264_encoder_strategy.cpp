#include "video_pipeline/nvenc_h264_encoder_strategy.h"

#include <cerrno>

namespace video_pipeline
{

int NvencH264EncoderStrategy::open(const VideoEncoderConfig & config)
{
  VideoEncoderConfig nvenc_config = config;
  nvenc_config.encoder_name = "h264_nvenc";
  nvenc_config.tune.clear();
  // 在 IDR 包里重复 SPS/PPS，让中途加入的解码节点也能收到参数。
  nvenc_config.global_header = false;

  AVDictionary * options = nullptr;
  // 关闭重排延迟和 FFmpeg 的额外输出等待。
  int result = av_dict_set(&options, "zerolatency", "1", 0);
  if (result >= 0) {
    result = av_dict_set(&options, "delay", "0", 0);
  }
  if (result >= 0) {
    result = encoder_.open(nvenc_config, &options);
  }
  av_dict_free(&options);
  return result;
}

int NvencH264EncoderStrategy::send_frame(const AVFrame * frame)
{
  return encoder_.send_frame(frame);
}

int NvencH264EncoderStrategy::receive_packet(AVPacket * packet)
{
  return encoder_.receive_packet(packet);
}

AVRational NvencH264EncoderStrategy::time_base() const
{
  return encoder_.time_base();
}

int NvencH264EncoderStrategy::set_bit_rate(std::int64_t)
{
  return AVERROR(ENOSYS);
}

}  // 结束 video_pipeline 命名空间
