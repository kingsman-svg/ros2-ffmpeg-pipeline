#ifndef VIDEO_PIPELINE_PCM_AUDIO_H_
#define VIDEO_PIPELINE_PCM_AUDIO_H_

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

extern "C" {
#include <libavutil/channel_layout.h>
#include <libavutil/intreadwrite.h>
#include <libavutil/mathematics.h>
}

#include "rclcpp/rclcpp.hpp"
#include "video_pipeline_msgs/msg/pcm_audio.hpp"

namespace video_pipeline
{

constexpr int kAudioRate = 48000;
constexpr int kAudioChannels = 2;
constexpr int kAudioBlockSamples = 960;
constexpr int kPcmBytesPerFrame = kAudioChannels * 2;

inline std::int64_t audio_time_ns(std::uint64_t sample_index)
{
  return av_rescale_q(static_cast<std::int64_t>(sample_index), {1, kAudioRate}, {1, 1000000000});
}

inline void append_s16le(
  std::vector<std::uint8_t> & output, const std::uint8_t * native_pcm, std::size_t byte_count)
{
  const auto offset = output.size();
  output.resize(offset + byte_count);
  for (std::size_t i = 0; i < byte_count; i += 2) {
    AV_WL16(output.data() + offset + i, AV_RN16(native_pcm + i));
  }
}

inline void validate_pcm(const video_pipeline_msgs::msg::PcmAudio & pcm)
{
  if (pcm.sample_rate != kAudioRate || pcm.channels != kAudioChannels ||
    pcm.sample_format != "s16le" || pcm.data.size() !=
    static_cast<std::uint64_t>(pcm.samples_per_channel) * kPcmBytesPerFrame ||
    pcm.samples_per_channel > kAudioBlockSamples ||
    pcm.end_of_stream != (pcm.samples_per_channel == 0))
  {
    throw std::invalid_argument("PCM 消息的格式或采样数无效");
  }
}

}  // 结束 video_pipeline 命名空间

#endif
