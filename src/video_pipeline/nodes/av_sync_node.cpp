#include <algorithm>
#include <chrono>
#include <deque>
#include <memory>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "video_pipeline/pcm_audio.h"

namespace
{

using video_pipeline_msgs::msg::PcmAudio;
using sensor_msgs::msg::Image;
using std_msgs::msg::Empty;
using std_msgs::msg::UInt64;
using Clock = std::chrono::steady_clock;
using Nanoseconds = std::chrono::nanoseconds;

constexpr std::size_t kMaxQueuedBlocks = 300;
constexpr std::size_t kMaxVideoBufferBytes = 256 * 1024 * 1024;

class AvSyncNode : public rclcpp::Node
{
public:
  AvSyncNode()
  : Node("av_sync"), waiting_since_(Clock::now())
  {
    start_ = create_publisher<Empty>("av/start", rclcpp::QoS(1).transient_local());
    audio_out_ = create_publisher<PcmAudio>("av/audio", rclcpp::QoS(50));
    video_out_ = create_publisher<Image>("av/video", rclcpp::QoS(30));
    video_end_out_ = create_publisher<UInt64>("av/video_end", rclcpp::QoS(1));
    audio_ready_ = create_subscription<Empty>(
      "audio/decoder_ready", rclcpp::QoS(1).transient_local(),
      [this](Empty::ConstSharedPtr) {decoder_ready_ = true;});
    audio_encoder_ready_ = create_subscription<Empty>(
      "audio/encoder_ready", rclcpp::QoS(1).transient_local(),
      [this](Empty::ConstSharedPtr) {encoder_ready_ = true;});
    audio_in_ = create_subscription<PcmAudio>(
      "audio/decoded_pcm", rclcpp::QoS(50),
      [this](PcmAudio::ConstSharedPtr message) {queue_audio(message);});
    video_in_ = create_subscription<Image>(
      "video/decoded_image", rclcpp::QoS(30),
      [this](Image::ConstSharedPtr message) {queue_video(message);});
    video_end_in_ = create_subscription<UInt64>(
      "video/end_of_stream", rclcpp::QoS(1), [this](UInt64::ConstSharedPtr message) {
        if (video_ended_ || message->data < received_video_) {
          throw std::runtime_error("视频结束帧数无效");
        }
        video_ended_ = true;
        video_total_ = message->data;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]() {tick();});
  }

private:
  void queue_audio(PcmAudio::ConstSharedPtr message)
  {
    video_pipeline::validate_pcm(*message);
    const auto stamp = rclcpp::Time(message->header.stamp).nanoseconds();
    if (audio_end_ || message->first_sample_index != received_samples_ || stamp <= last_audio_ns_) {
      throw std::runtime_error("同步节点收到的音频采样序号或时间戳无效");
    }
    last_audio_ns_ = stamp;
    last_audio_received_at_ = Clock::now();
    if (message->end_of_stream) {
      audio_end_ = message;
      return;
    }
    if (audio_queue_.size() >= kMaxQueuedBlocks) {
      throw std::runtime_error("同步节点的音频缓冲已满");
    }
    received_samples_ += message->samples_per_channel;
    audio_queue_.push_back(message);
  }

  void queue_video(Image::ConstSharedPtr message)
  {
    const auto stamp = rclcpp::Time(message->header.stamp).nanoseconds();
    if (stamp < last_video_ns_ || (video_ended_ && received_video_ >= video_total_)) {
      throw std::runtime_error("同步节点收到的视频时间戳或帧数无效");
    }
    if (video_queue_.size() >= kMaxQueuedBlocks ||
      message->data.size() > kMaxVideoBufferBytes - video_buffer_bytes_)
    {
      throw std::runtime_error("同步节点的视频缓冲已满");
    }
    last_video_ns_ = stamp;
    last_video_received_at_ = Clock::now();
    ++received_video_;
    video_buffer_bytes_ += message->data.size();
    video_queue_.push_back(message);
  }

  void tick()
  {
    const auto now = Clock::now();
    if (!sources_started_) {
      if (decoder_ready_ && encoder_ready_ && video_in_->get_publisher_count() > 0 &&
        video_end_in_->get_publisher_count() > 0 && start_->get_subscription_count() >= 2)
      {
        start_->publish(Empty{});
        sources_started_ = true;
        waiting_since_ = now;
      } else if (now - waiting_since_ >= std::chrono::seconds(10)) {
        throw std::runtime_error("等待音视频链路就绪超时");
      }
      return;
    }

    const bool video_complete = video_ended_ && received_video_ == video_total_;
    if (!clock_started_) {
      if ((audio_queue_.empty() && !audio_end_) || (video_queue_.empty() && !video_complete)) {
        if (now - waiting_since_ >= std::chrono::seconds(10)) {
          throw std::runtime_error("同步节点等待首批音视频数据超时");
        }
        return;
      }
      clock_started_ = true;
      last_tick_ = now + std::chrono::milliseconds(200);
      return;
    }
    if (now < last_tick_) {
      return;
    }
    const auto next_playhead = playhead_ + (now - last_tick_);
    if (audio_queue_.empty() && !audio_end_) {
      if (now - last_audio_received_at_ >= std::chrono::seconds(10)) {
        throw std::runtime_error("同步节点等待音频数据超时");
      }
      // 播放时钟可以走完已发布的采样。视频的帧间停顿仍沿用同一时钟。
      playhead_ = std::max(playhead_, std::min(next_playhead, audio_available_until_));
    } else {
      playhead_ = next_playhead;
    }
    last_tick_ = now;
    if (video_ended_ && !video_complete &&
      now - last_video_received_at_ >= std::chrono::seconds(10))
    {
      throw std::runtime_error("视频结束通知后的解码帧未收齐");
    }

    while (!audio_queue_.empty() || !video_queue_.empty()) {
      const bool use_audio = !audio_queue_.empty() &&
        (video_queue_.empty() || rclcpp::Time(audio_queue_.front()->header.stamp).nanoseconds() <=
        rclcpp::Time(video_queue_.front()->header.stamp).nanoseconds());
      if (use_audio) {
        const auto & message = audio_queue_.front();
        const auto stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
        if (stamp_ns > playhead_.count()) {
          break;
        }
        audio_out_->publish(*message);
        audio_available_until_ = Nanoseconds(
          stamp_ns + video_pipeline::audio_time_ns(message->samples_per_channel));
        audio_queue_.pop_front();
      } else {
        const auto & message = video_queue_.front();
        if (rclcpp::Time(message->header.stamp).nanoseconds() > playhead_.count()) {
          break;
        }
        video_out_->publish(*message);
        video_buffer_bytes_ -= message->data.size();
        video_queue_.pop_front();
      }
    }

    if (audio_end_ && video_complete && audio_queue_.empty() && video_queue_.empty() &&
      rclcpp::Time(audio_end_->header.stamp).nanoseconds() <= playhead_.count())
    {
      audio_out_->publish(*audio_end_);
      UInt64 end;
      end.data = video_total_;
      video_end_out_->publish(end);
      const auto timeout = std::chrono::seconds(2);
      const bool audio_acked = audio_out_->wait_for_all_acked(timeout);
      const bool video_acked = video_out_->wait_for_all_acked(timeout);
      const bool end_acked = video_end_out_->wait_for_all_acked(timeout);
      if (!audio_acked || !video_acked || !end_acked) {
        RCLCPP_WARN(get_logger(), "等待同步输出确认超时");
      }
      RCLCPP_INFO_STREAM(get_logger(), "音视频同步结束，发布 " << video_total_ <<
        " 帧和每声道 " << received_samples_ << " 个采样");
      rclcpp::shutdown();
    }
  }

  std::deque<PcmAudio::ConstSharedPtr> audio_queue_;
  std::deque<Image::ConstSharedPtr> video_queue_;
  PcmAudio::ConstSharedPtr audio_end_;
  std::size_t video_buffer_bytes_ = 0;
  std::uint64_t received_samples_ = 0;
  std::uint64_t received_video_ = 0;
  std::uint64_t video_total_ = 0;
  std::int64_t last_audio_ns_ = -1;
  std::int64_t last_video_ns_ = -1;
  bool video_ended_ = false;
  bool decoder_ready_ = false;
  bool encoder_ready_ = false;
  bool sources_started_ = false;
  bool clock_started_ = false;
  Nanoseconds playhead_{0};
  Nanoseconds audio_available_until_{0};
  Clock::time_point last_tick_;
  Clock::time_point waiting_since_;
  Clock::time_point last_audio_received_at_;
  Clock::time_point last_video_received_at_;
  rclcpp::Publisher<Empty>::SharedPtr start_;
  rclcpp::Publisher<PcmAudio>::SharedPtr audio_out_;
  rclcpp::Publisher<Image>::SharedPtr video_out_;
  rclcpp::Publisher<UInt64>::SharedPtr video_end_out_;
  rclcpp::Subscription<Empty>::SharedPtr audio_ready_;
  rclcpp::Subscription<Empty>::SharedPtr audio_encoder_ready_;
  rclcpp::Subscription<PcmAudio>::SharedPtr audio_in_;
  rclcpp::Subscription<Image>::SharedPtr video_in_;
  rclcpp::Subscription<UInt64>::SharedPtr video_end_in_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // 结束匿名命名空间

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AvSyncNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("av_sync"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
