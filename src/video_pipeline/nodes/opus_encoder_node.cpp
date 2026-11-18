#include <algorithm>
#include <cerrno>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
}

#include "video_pipeline/ffmpeg_resources.h"
#include "video_pipeline/pcm_audio.h"
#include "std_msgs/msg/empty.hpp"
#include "video_pipeline_msgs/msg/opus_packet.hpp"

namespace
{

using namespace video_pipeline;
using video_pipeline_msgs::msg::OpusPacket;
using video_pipeline_msgs::msg::PcmAudio;

void free_context(AVCodecContext * context)
{
  avcodec_free_context(&context);
}

class OpusEncoderNode : public rclcpp::Node
{
public:
  OpusEncoderNode()
  : Node("opus_encoder"), encoder_(nullptr, free_context), frame_(av_frame_alloc()),
    packet_(av_packet_alloc())
  {
    const auto bit_rate = declare_parameter<std::int64_t>("bit_rate", 64000);
    const auto * codec = avcodec_find_encoder_by_name("libopus");
    if (codec == nullptr) {
      throw std::runtime_error("当前 FFmpeg 没有 libopus 编码器");
    }
    encoder_.reset(avcodec_alloc_context3(codec));
    if (!encoder_ || !frame_ || !packet_ || bit_rate <= 0) {
      throw std::invalid_argument("Opus 码率或编码缓冲无效");
    }
    encoder_->sample_fmt = AV_SAMPLE_FMT_S16;
    encoder_->sample_rate = kAudioRate;
    encoder_->channel_layout = AV_CH_LAYOUT_STEREO;
    encoder_->channels = kAudioChannels;
    encoder_->bit_rate = bit_rate;
    encoder_->time_base = {1, kAudioRate};
    int result = av_opt_set(encoder_->priv_data, "frame_duration", "20", 0);
    if (result >= 0) {
      result = avcodec_open2(encoder_.get(), codec, nullptr);
    }
    if (result < 0 || encoder_->frame_size != kAudioBlockSamples) {
      throw ffmpeg_error("打开 Opus 编码器失败", result < 0 ? result : AVERROR(EINVAL));
    }
    frame_->format = encoder_->sample_fmt;
    frame_->sample_rate = kAudioRate;
    frame_->channel_layout = AV_CH_LAYOUT_STEREO;
    frame_->channels = kAudioChannels;
    frame_->nb_samples = kAudioBlockSamples;
    result = av_frame_get_buffer(frame_.get(), 0);
    if (result < 0) {
      throw ffmpeg_error("分配 Opus 输入帧失败", result);
    }

    publisher_ = create_publisher<OpusPacket>("audio/opus", rclcpp::QoS(50));
    subscription_ = create_subscription<PcmAudio>(
      "audio/pcm_raw", rclcpp::QoS(50), [this](PcmAudio::ConstSharedPtr pcm) {accept_pcm(*pcm);});
    if (declare_parameter<bool>("announce_ready", false)) {
      ready_publisher_ = create_publisher<std_msgs::msg::Empty>(
        "audio/encoder_ready", rclcpp::QoS(1).transient_local());
      ready_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
        if (subscription_->get_publisher_count() > 0 && publisher_->get_subscription_count() > 0) {
          ready_publisher_->publish(std_msgs::msg::Empty{});
          ready_timer_->cancel();
        }
      });
    }
  }

private:
  void accept_pcm(const PcmAudio & pcm)
  {
    validate_pcm(pcm);
    if (pcm.first_sample_index != received_samples_) {
      throw std::runtime_error("原始 PCM 的采样位置不连续，期望 " +
        std::to_string(received_samples_) + "，实际 " + std::to_string(pcm.first_sample_index));
    }
    if (received_samples_ == 0) {
      origin_ns_ = rclcpp::Time(pcm.header.stamp).nanoseconds();
      frame_id_ = pcm.header.frame_id;
    }
    if (pcm.end_of_stream) {
      finish();
      return;
    }
    received_samples_ += pcm.samples_per_channel;
    pending_.insert(pending_.end(), pcm.data.begin(), pcm.data.end());
    constexpr auto block_bytes = kAudioBlockSamples * kPcmBytesPerFrame;
    if (pending_.size() >= block_bytes) {
      encode_block(kAudioBlockSamples);
      pending_.erase(pending_.begin(), pending_.begin() + block_bytes);
    }
  }

  void encode_block(int samples)
  {
    frame_->nb_samples = samples;
    int result = av_frame_make_writable(frame_.get());
    if (result < 0) {
      throw ffmpeg_error("使 Opus 输入帧可写失败", result);
    }
    for (int i = 0; i < samples * kPcmBytesPerFrame; i += 2) {
      AV_WN16(frame_->data[0] + i, AV_RL16(pending_.data() + i));
    }
    frame_->pts = submitted_samples_;
    result = avcodec_send_frame(encoder_.get(), frame_.get());
    if (result < 0) {
      throw ffmpeg_error("送入 Opus 编码帧失败", result);
    }
    submitted_samples_ += samples;
    publish_packets(AVERROR(EAGAIN));
  }

  void publish_packets(int expected_result)
  {
    while (true) {
      const int result = avcodec_receive_packet(encoder_.get(), packet_.get());
      if (result == expected_result) {
        return;
      }
      if (result < 0) {
        throw ffmpeg_error("取出 Opus 编码包失败", result);
      }
      if (packet_->pts == AV_NOPTS_VALUE || packet_->size <= 0 ||
        packet_->duration <= 0 || packet_->duration > kAudioBlockSamples)
      {
        throw std::runtime_error("Opus 编码包的时间戳或时长无效");
      }
      // 首包 PTS 可能为负，尾包的有效时长由编码器给出。
      const auto first = std::max<std::int64_t>(0, packet_->pts);
      const auto end = std::min(submitted_samples_, packet_->pts + packet_->duration);
      OpusPacket message;
      message.header.stamp = rclcpp::Time(origin_ns_ + audio_time_ns(first));
      message.header.frame_id = frame_id_;
      message.first_sample_index = first;
      message.skip_samples = first - packet_->pts;
      message.samples_per_channel = std::max<std::int64_t>(0, end - first);
      message.data.assign(packet_->data, packet_->data + packet_->size);
      publisher_->publish(message);
    }
  }

  void finish()
  {
    if (!pending_.empty()) {
      encode_block(pending_.size() / kPcmBytesPerFrame);
    }
    if (submitted_samples_ > 0) {
      const int result = avcodec_send_frame(encoder_.get(), nullptr);
      if (result < 0) {
        throw ffmpeg_error("排空 Opus 编码器失败", result);
      }
      publish_packets(AVERROR_EOF);
    }
    OpusPacket end;
    end.header.stamp = rclcpp::Time(origin_ns_ + audio_time_ns(received_samples_));
    end.header.frame_id = frame_id_;
    end.first_sample_index = received_samples_;
    end.end_of_stream = true;
    publisher_->publish(end);
    if (!publisher_->wait_for_all_acked(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "等待 Opus 结束消息确认超时");
    }
    RCLCPP_INFO_STREAM(get_logger(), "Opus 编码结束，每声道 " << received_samples_ << " 个采样");
    rclcpp::shutdown();
  }

  std::unique_ptr<AVCodecContext, decltype(&free_context)> encoder_;
  std::unique_ptr<AVFrame, FrameDeleter> frame_;
  std::unique_ptr<AVPacket, PacketDeleter> packet_;
  std::vector<std::uint8_t> pending_;
  std::uint64_t received_samples_ = 0;
  std::int64_t submitted_samples_ = 0;
  std::int64_t origin_ns_ = 0;
  std::string frame_id_;
  rclcpp::Publisher<OpusPacket>::SharedPtr publisher_;
  rclcpp::Subscription<PcmAudio>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ready_publisher_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
};

}  // 结束匿名命名空间

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<OpusEncoderNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("opus_encoder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
