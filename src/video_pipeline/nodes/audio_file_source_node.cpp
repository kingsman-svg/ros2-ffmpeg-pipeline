#include <algorithm>
#include <cerrno>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "video_pipeline/audio_resampler.h"
#include "video_pipeline/ffmpeg_resources.h"
#include "video_pipeline/media_decoder.h"
#include "video_pipeline/media_reader.h"
#include "video_pipeline/pcm_audio.h"

namespace
{

using namespace video_pipeline;
using video_pipeline_msgs::msg::PcmAudio;
using Clock = std::chrono::steady_clock;

class AudioFileSourceNode : public rclcpp::Node
{
public:
  AudioFileSourceNode()
  : Node("audio_file_source"), packet_(av_packet_alloc()), frame_(av_frame_alloc()),
    converted_(av_frame_alloc())
  {
    const auto path = declare_parameter<std::string>("input_path", "");
    if (path.empty() || !packet_ || !frame_ || !converted_) {
      throw std::invalid_argument("需要有效的音频输入路径及解码缓冲");
    }
    int result = reader_.open(path, AVMEDIA_TYPE_AUDIO);
    if (result < 0) {
      throw ffmpeg_error("打开音轨失败 " + path, result);
    }
    result = decoder_.open(reader_.codec_parameters());
    if (result < 0) {
      throw ffmpeg_error("初始化文件音频解码器失败", result);
    }
    publisher_ = create_publisher<PcmAudio>("audio/pcm_raw", rclcpp::QoS(50));
    ready_ = !declare_parameter<bool>("wait_for_av_start", false);
    if (!ready_) {
      start_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "av/start", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::Empty::ConstSharedPtr) {ready_ = true;});
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]() {publish_block();});
  }

private:
  void fill_pcm()
  {
    constexpr auto block_bytes = kAudioBlockSamples * kPcmBytesPerFrame;
    while (buffer_.size() - read_offset_ < block_bytes && !finished_ && rclcpp::ok()) {
      if (read_offset_ > 0) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + read_offset_);
        read_offset_ = 0;
      }
      if (decoder_finished_) {
        if (!resampler_ready_) {
          throw std::runtime_error("音轨没有可解码的采样");
        }
        const int samples = resampler_.convert(nullptr, converted_.get());
        if (samples < 0) {
          throw ffmpeg_error("排空音频重采样失败", samples);
        }
        finished_ = samples == 0;
        if (samples > 0) {
          append_s16le(buffer_, converted_->data[0], samples * std::size_t{kPcmBytesPerFrame});
        }
        continue;
      }

      int result = decoder_.receive_frame(frame_.get());
      if (result == AVERROR_EOF) {
        decoder_finished_ = true;
        continue;
      }
      if (result == AVERROR(EAGAIN)) {
        result = reader_.read_packet(packet_.get());
        if (result < 0 && result != AVERROR_EOF) {
          throw ffmpeg_error("读取音频包失败", result);
        }
        result = decoder_.send_packet(result == AVERROR_EOF ? nullptr : packet_.get());
        if (result < 0) {
          throw ffmpeg_error("送入文件音频包失败", result);
        }
        continue;
      }
      if (result < 0) {
        throw ffmpeg_error("解码文件音频失败", result);
      }
      resample_frame();
    }
  }

  void resample_frame()
  {
    if (!resampler_ready_) {
      auto layout = static_cast<std::int64_t>(frame_->channel_layout);
      if (layout == 0) {
        layout = av_get_default_channel_layout(frame_->channels);
      }
      const int result = resampler_.open(
        frame_->sample_rate, static_cast<AVSampleFormat>(frame_->format), layout,
        kAudioRate, AV_SAMPLE_FMT_S16, AV_CH_LAYOUT_STEREO);
      if (result < 0) {
        throw ffmpeg_error("初始化音频重采样失败", result);
      }
      origin_ns_ = reader_.media_time_ns(frame_->best_effort_timestamp);
      resampler_ready_ = true;
    }
    const int samples = resampler_.convert(frame_.get(), converted_.get());
    if (samples < 0) {
      throw ffmpeg_error("重采样文件音频失败", samples);
    }
    if (samples > 0) {
      append_s16le(buffer_, converted_->data[0], samples * std::size_t{kPcmBytesPerFrame});
    }
  }

  void publish_block()
  {
    if (!started_) {
      if (!ready_ || publisher_->get_subscription_count() == 0) {
        return;
      }
      started_ = true;
      started_at_ = Clock::now();
    }
    fill_pcm();
    if (!rclcpp::ok()) {
      return;
    }

    const auto stamp_ns = origin_ns_ + audio_time_ns(published_samples_);
    if (published_samples_ > 0 && Clock::now() - started_at_ < std::chrono::nanoseconds(stamp_ns)) {
      return;
    }
    const auto bytes = std::min<std::size_t>(
      buffer_.size() - read_offset_, kAudioBlockSamples * kPcmBytesPerFrame);
    PcmAudio message;
    message.header.stamp = rclcpp::Time(stamp_ns);
    message.header.frame_id = "audio";
    message.sample_rate = kAudioRate;
    message.channels = kAudioChannels;
    message.sample_format = "s16le";
    message.first_sample_index = published_samples_;
    message.samples_per_channel = bytes / kPcmBytesPerFrame;
    message.end_of_stream = bytes == 0;
    message.data.assign(buffer_.begin() + read_offset_, buffer_.begin() + read_offset_ + bytes);
    publisher_->publish(message);
    read_offset_ += bytes;
    published_samples_ += message.samples_per_channel;
    if (message.end_of_stream) {
      timer_->cancel();
      if (!publisher_->wait_for_all_acked(std::chrono::seconds(2))) {
        RCLCPP_WARN(get_logger(), "等待 PCM 结束消息确认超时");
      }
      RCLCPP_INFO_STREAM(get_logger(), "PCM 文件发送结束，每声道 " << published_samples_ << " 个采样");
      rclcpp::shutdown();
    }
  }

  MediaReader reader_;
  MediaDecoder decoder_;
  AudioResampler resampler_;
  std::unique_ptr<AVPacket, PacketDeleter> packet_;
  std::unique_ptr<AVFrame, FrameDeleter> frame_;
  std::unique_ptr<AVFrame, FrameDeleter> converted_;
  std::vector<std::uint8_t> buffer_;
  std::size_t read_offset_ = 0;
  std::uint64_t published_samples_ = 0;
  std::int64_t origin_ns_ = 0;
  bool resampler_ready_ = false;
  bool decoder_finished_ = false;
  bool finished_ = false;
  bool ready_ = false;
  bool started_ = false;
  Clock::time_point started_at_;
  rclcpp::Publisher<PcmAudio>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // 结束匿名命名空间

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AudioFileSourceNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_file_source"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
