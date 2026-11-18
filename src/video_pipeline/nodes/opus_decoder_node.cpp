#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>

#include "std_msgs/msg/empty.hpp"
#include "video_pipeline/audio_resampler.h"
#include "video_pipeline/ffmpeg_resources.h"
#include "video_pipeline/media_decoder.h"
#include "video_pipeline/pcm_audio.h"
#include "video_pipeline_msgs/msg/opus_packet.hpp"

namespace
{

using namespace video_pipeline;
using video_pipeline_msgs::msg::OpusPacket;
using video_pipeline_msgs::msg::PcmAudio;

class OpusDecoderNode : public rclcpp::Node
{
public:
  OpusDecoderNode()
  : Node("opus_decoder"), packet_(av_packet_alloc()), frame_(av_frame_alloc()),
    converted_(av_frame_alloc())
  {
    if (!packet_ || !frame_ || !converted_) {
      throw std::runtime_error("分配 Opus 解码缓冲失败");
    }
    AVCodecParameters parameters{};
    parameters.codec_type = AVMEDIA_TYPE_AUDIO;
    parameters.codec_id = AV_CODEC_ID_OPUS;
    parameters.sample_rate = kAudioRate;
    parameters.channels = kAudioChannels;
    parameters.channel_layout = AV_CH_LAYOUT_STEREO;
    const int result = decoder_.open(&parameters);
    if (result < 0) {
      throw ffmpeg_error("打开 Opus 解码器失败", result);
    }

    publisher_ = create_publisher<PcmAudio>("audio/decoded_pcm", rclcpp::QoS(50));
    subscription_ = create_subscription<OpusPacket>(
      "audio/opus", rclcpp::QoS(50),
      [this](OpusPacket::ConstSharedPtr message) {decode_packet(*message);});
    if (declare_parameter<bool>("announce_ready", false)) {
      ready_publisher_ = create_publisher<std_msgs::msg::Empty>(
        "audio/decoder_ready", rclcpp::QoS(1).transient_local());
      ready_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
        if (subscription_->get_publisher_count() > 0 && publisher_->get_subscription_count() > 0) {
          ready_publisher_->publish(std_msgs::msg::Empty{});
          ready_timer_->cancel();
        }
      });
    }
  }

private:
  void decode_packet(const OpusPacket & message)
  {
    if (message.first_sample_index != published_samples_) {
      throw std::runtime_error("Opus 消息的有效采样位置不连续");
    }
    PcmAudio pcm;
    pcm.header = message.header;
    pcm.sample_rate = kAudioRate;
    pcm.channels = kAudioChannels;
    pcm.sample_format = "s16le";
    pcm.first_sample_index = published_samples_;
    if (message.end_of_stream) {
      if (!message.data.empty() || message.skip_samples != 0 || message.samples_per_channel != 0) {
        throw std::runtime_error("Opus 结束消息携带了数据");
      }
      int result = decoder_.send_packet(nullptr);
      if (result >= 0) {
        result = decoder_.receive_frame(frame_.get());
      }
      if (result != AVERROR_EOF) {
        throw ffmpeg_error("排空 Opus 解码器失败", result < 0 ? result : AVERROR_BUG);
      }
      pcm.end_of_stream = true;
      publisher_->publish(pcm);
      if (!publisher_->wait_for_all_acked(std::chrono::seconds(2))) {
        RCLCPP_WARN(get_logger(), "等待解码 PCM 结束消息确认超时");
      }
      RCLCPP_INFO_STREAM(get_logger(), "Opus 解码结束，每声道 " << published_samples_ << " 个采样");
      rclcpp::shutdown();
      return;
    }
    if (message.data.empty() || message.data.size() > 8192 ||
      message.skip_samples > kAudioBlockSamples ||
      message.samples_per_channel > kAudioBlockSamples - message.skip_samples)
    {
      throw std::invalid_argument("Opus 包的长度或采样范围无效");
    }

    av_packet_unref(packet_.get());
    int result = av_new_packet(packet_.get(), static_cast<int>(message.data.size()));
    if (result < 0) {
      throw ffmpeg_error("分配 Opus 数据包失败", result);
    }
    std::memcpy(packet_->data, message.data.data(), message.data.size());
    result = decoder_.send_packet(packet_.get());
    if (result >= 0) {
      result = decoder_.receive_frame(frame_.get());
    }
    if (result < 0) {
      throw ffmpeg_error("解码 Opus 数据包失败", result);
    }
    if (frame_->nb_samples != kAudioBlockSamples || frame_->sample_rate != kAudioRate ||
      frame_->channels != kAudioChannels)
    {
      throw std::runtime_error("Opus 解码样本格式与传输约定不符");
    }
    if (!resampler_ready_) {
      result = resampler_.open(
        kAudioRate, static_cast<AVSampleFormat>(frame_->format), AV_CH_LAYOUT_STEREO,
        kAudioRate, AV_SAMPLE_FMT_S16, AV_CH_LAYOUT_STEREO);
      if (result < 0) {
        throw ffmpeg_error("初始化 Opus PCM 转换失败", result);
      }
      resampler_ready_ = true;
    }
    result = resampler_.convert(frame_.get(), converted_.get());
    if (result != kAudioBlockSamples) {
      throw ffmpeg_error("转换 Opus PCM 失败", result < 0 ? result : AVERROR_BUG);
    }
    if (message.samples_per_channel > 0) {
      // 解码整包以维持编解码状态，仅向下游发布有效的采样。
      pcm.samples_per_channel = message.samples_per_channel;
      append_s16le(
        pcm.data, converted_->data[0] + message.skip_samples * kPcmBytesPerFrame,
        message.samples_per_channel * kPcmBytesPerFrame);
      publisher_->publish(pcm);
      published_samples_ += message.samples_per_channel;
    }
  }

  MediaDecoder decoder_;
  AudioResampler resampler_;
  std::unique_ptr<AVPacket, PacketDeleter> packet_;
  std::unique_ptr<AVFrame, FrameDeleter> frame_;
  std::unique_ptr<AVFrame, FrameDeleter> converted_;
  std::uint64_t published_samples_ = 0;
  bool resampler_ready_ = false;
  rclcpp::Publisher<PcmAudio>::SharedPtr publisher_;
  rclcpp::Subscription<OpusPacket>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ready_publisher_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
};

}  // 结束匿名命名空间

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<OpusDecoderNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("opus_decoder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
