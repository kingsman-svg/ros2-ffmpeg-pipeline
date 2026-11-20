#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/header.hpp"

#include "video_pipeline/ffmpeg_resources.h"
#include "video_pipeline/h264_annex_b_parser.h"
#include "video_pipeline/h264_decoder_strategy.h"
#include "video_pipeline/h264_packet_queue.h"
#include "video_pipeline/image_converter.h"
#include "video_pipeline/nvdec_h264_decoder_strategy.h"
#include "video_pipeline/rgb_image_message.h"
#include "video_pipeline_msgs/msg/h264_packet.hpp"
#include "video_pipeline_msgs/msg/h264_queue_feedback.hpp"

namespace
{

using sensor_msgs::msg::Image;
using video_pipeline::ffmpeg_error;
using video_pipeline::FrameDeleter;
using video_pipeline::H264PacketQueue;
using video_pipeline::PacketDeleter;
using video_pipeline_msgs::msg::H264Packet;
using video_pipeline_msgs::msg::H264QueueFeedback;
using SteadyClock = std::chrono::steady_clock;

struct DecodeStageStats
{
  std::uint64_t published_frames = 0;
  SteadyClock::duration decode_time{};
  SteadyClock::duration convert_time{};
  SteadyClock::duration publish_time{};
};

std::uint64_t to_microseconds(SteadyClock::duration duration)
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
}

class H264DecoderNode : public rclcpp::Node
{
public:
  H264DecoderNode()
  : Node("h264_decoder"), packet_(av_packet_alloc()), decoded_frame_(av_frame_alloc()),
    rgb_frame_(av_frame_alloc())
  {
    if (!packet_ || !decoded_frame_ || !rgb_frame_) {
      throw std::runtime_error("分配解码数据结构失败");
    }

    rcl_interfaces::msg::ParameterDescriptor startup_parameter;
    startup_parameter.read_only = true;
    const auto backend = declare_parameter<std::string>("decoder_backend", "auto", startup_parameter);
    if (backend != "auto" && backend != "software" && backend != "nvdec") {
      throw std::invalid_argument("decoder_backend 必须是 auto、software 或 nvdec");
    }
    const auto max_packets = declare_parameter<std::int64_t>("queue_max_packets", 120, startup_parameter);
    const auto max_bytes = declare_parameter<std::int64_t>("queue_max_bytes", 8 * 1024 * 1024, startup_parameter);
    decode_queue_ = std::make_unique<H264PacketQueue>("H.264 解码队列", max_packets, max_bytes);

    open_decoder_strategy(backend);

    publisher_ = create_publisher<Image>("video/decoded_image", rclcpp::QoS(30));
    subscription_ = create_subscription<H264Packet>(
      "video/h264", rclcpp::QoS(30),
      [this](H264Packet::ConstSharedPtr message) {
        if (message->end_of_stream) {
          decode_queue_->close();
          return;
        }
        decode_queue_->push(std::move(message));
      });
    feedback_publisher_ = create_publisher<H264QueueFeedback>(
      "video/queue_feedback", rclcpp::QoS(1).best_effort().durability_volatile());
    feedback_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() {publish_queue_feedback();});
    if (declare_parameter<bool>("announce_ready", false)) {
      // 保留就绪消息，文件源稍后启动也能收到。
      ready_publisher_ = create_publisher<std_msgs::msg::Empty>(
        "video/decoder_ready", rclcpp::QoS(1).transient_local());
      ready_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (subscription_->get_publisher_count() > 0) {
          ready_publisher_->publish(std_msgs::msg::Empty{});
          ready_timer_->cancel();
        }
      });
    }
    stats_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      RCLCPP_INFO_STREAM(get_logger(), "H.264 解码队列：" << decode_queue_->stats());
    });
    RCLCPP_INFO_STREAM(
      get_logger(), "等待 " << subscription_->get_topic_name()
                             << "，解码图像输出到 " << publisher_->get_topic_name());
    decode_thread_ = std::thread([this]() {
      try {
        decode_queued_packets();
      } catch (...) {
        if (rclcpp::ok()) {
          decode_error_ = std::current_exception();
          rclcpp::shutdown();
        }
      }
    });
  }

  ~H264DecoderNode() override
  {
    stop_decode_thread();
  }

  void check_decode_result()
  {
    stop_decode_thread();
    if (decode_error_) {
      std::rethrow_exception(decode_error_);
    }
  }

private:
  void open_decoder_strategy(const std::string & backend)
  {
    if (backend != "software") {
      decoder_strategy_.reset(new video_pipeline::NvdecH264DecoderStrategy(backend == "auto"));
      const int result = decoder_strategy_->open();
      if (result >= 0) {
        RCLCPP_INFO(get_logger(), "NVDEC 的 CUDA 解码上下文已就绪，等待首帧");
        return;
      }
      const auto error = ffmpeg_error("打开 NVDEC 解码器失败", result);
      if (backend == "nvdec") {
        throw error;
      }
      RCLCPP_WARN_STREAM(get_logger(), error.what() << "，回退到软件解码");
    }

    decoder_strategy_.reset(new video_pipeline::SoftwareH264DecoderStrategy());
    const int result = decoder_strategy_->open();
    if (result < 0) {
      throw ffmpeg_error("打开 H.264 软件解码器失败", result);
    }
    RCLCPP_INFO(get_logger(), "H.264 解码后端为软件解码");
  }

  void publish_queue_feedback()
  {
    DecodeStageStats stage_stats;
    {
      std::lock_guard<std::mutex> lock(stage_stats_mutex_);
      stage_stats = total_stage_stats_;
    }
    const auto queue_stats = decode_queue_->stats();
    H264QueueFeedback feedback;
    feedback.packet_count = queue_stats.packet_count;
    feedback.max_packets = queue_stats.max_packets;
    feedback.byte_count = queue_stats.byte_count;
    feedback.max_bytes = queue_stats.max_bytes;
    feedback.oldest_wait_ms = static_cast<std::uint64_t>(queue_stats.oldest_wait.count());
    feedback.sample_elapsed_us = to_microseconds(SteadyClock::now() - started_at_);
    feedback.total_enqueued_packets = queue_stats.total_enqueued_packets;
    feedback.total_dequeued_packets = queue_stats.total_dequeued_packets;
    feedback.total_published_frames = stage_stats.published_frames;
    feedback.total_decode_time_us = to_microseconds(stage_stats.decode_time);
    feedback.total_convert_time_us = to_microseconds(stage_stats.convert_time);
    feedback.total_publish_time_us = to_microseconds(stage_stats.publish_time);
    feedback_publisher_->publish(feedback);
  }

  void accumulate_stage_stats(const DecodeStageStats & stats)
  {
    // 解码线程完成一批操作后才合并耗时，避免计时和发布时持锁。
    std::lock_guard<std::mutex> lock(stage_stats_mutex_);
    total_stage_stats_.published_frames += stats.published_frames;
    total_stage_stats_.decode_time += stats.decode_time;
    total_stage_stats_.convert_time += stats.convert_time;
    total_stage_stats_.publish_time += stats.publish_time;
  }

  void stop_decode_thread()
  {
    if (decode_thread_.joinable()) {
      decode_queue_->cancel();
      decode_thread_.join();
    }
  }

  void decode_queued_packets()
  {
    H264PacketQueue::Packet packet;
    while (rclcpp::ok()) {
      const auto result = decode_queue_->wait_pop(packet);
      if (result == H264PacketQueue::PopResult::Cancelled || !rclcpp::ok()) {
        return;
      }
      if (result == H264PacketQueue::PopResult::Finished) {
        // 结束标记关闭队列，消费完之前入队的包后再排空解码器。
        finish_stream();
        return;
      }
      decode_packet(*packet);
    }
  }

  void decode_packet(const H264Packet & message)
  {
    video_pipeline::H264AnnexBParser::StartPacketPlan start_plan;
    if (!decoding_started_) {
      start_plan = bitstream_parser_.inspect_start_packet(message.data);
      if (!start_plan.ready) {
        return;
      }
    }
    std::size_t parameter_size = 0;
    if (start_plan.sps != nullptr) {
      parameter_size = start_plan.sps->size() + start_plan.pps->size();
    }
    const auto max_packet_size = static_cast<std::size_t>(
      std::numeric_limits<int>::max() - AV_INPUT_BUFFER_PADDING_SIZE);
    if (message.data.empty() || message.data.size() > max_packet_size ||
      parameter_size > max_packet_size - message.data.size() || message.pts == AV_NOPTS_VALUE)
    {
      throw std::invalid_argument("H.264 数据包大小或 PTS 无效");
    }

    // av_new_packet 会在数据末尾分配 FFmpeg 解码所需的零填充。
    av_packet_unref(packet_.get());
    const int allocation_result = av_new_packet(
      packet_.get(), static_cast<int>(message.data.size() + parameter_size));
    if (allocation_result < 0) {
      throw ffmpeg_error("分配 H.264 数据包失败", allocation_result);
    }
    if (parameter_size == 0) {
      std::memcpy(packet_->data, message.data.data(), message.data.size());
    } else {
      // 按解析器给出的顺序把参数直接写入最终缓冲区。
      const std::size_t offset = start_plan.insertion_offset;
      std::memcpy(packet_->data, message.data.data(), offset);
      std::memcpy(packet_->data + offset, start_plan.sps->data(), start_plan.sps->size());
      std::memcpy(
        packet_->data + offset + start_plan.sps->size(), start_plan.pps->data(),
        start_plan.pps->size());
      std::memcpy(
        packet_->data + offset + parameter_size, message.data.data() + offset,
        message.data.size() - offset);
    }
    packet_->pts = message.pts;
    packet_->dts = message.dts;
    if (!decoding_started_ || message.key_frame) {
      packet_->flags |= AV_PKT_FLAG_KEY;
    }

    DecodeStageStats stats;
    const auto decode_started_at = SteadyClock::now();
    const int send_result = decoder_strategy_->send_packet(packet_.get());
    stats.decode_time += SteadyClock::now() - decode_started_at;
    if (send_result < 0) {
      throw ffmpeg_error("送入 H.264 数据包失败", send_result);
    }
    decoding_started_ = true;
    // 解码可能延迟或重排，按 PTS 找回每帧自己的 ROS 头部。
    if (!input_headers_.emplace(message.pts, message.header).second) {
      throw std::invalid_argument("收到重复的 H.264 PTS");
    }
    const int receive_result = publish_frames(stats);
    if (receive_result != AVERROR(EAGAIN)) {
      throw ffmpeg_error("读取解码帧失败", receive_result);
    }
    accumulate_stage_stats(stats);
  }

  int publish_frames(DecodeStageStats & stats)
  {
    while (true) {
      const auto decode_started_at = SteadyClock::now();
      const int result = decoder_strategy_->receive_frame(decoded_frame_.get());
      stats.decode_time += SteadyClock::now() - decode_started_at;
      if (result < 0) {
        return result;
      }
      const auto header = input_headers_.find(decoded_frame_->pts);
      if (header == input_headers_.end()) {
        throw std::runtime_error("找不到解码帧对应的 ROS 头部");
      }
      const auto convert_started_at = SteadyClock::now();
      const int converted = converter_.convert(decoded_frame_.get(), AV_PIX_FMT_RGB24, rgb_frame_.get());
      stats.convert_time += SteadyClock::now() - convert_started_at;
      if (converted < 0) {
        throw ffmpeg_error("转换 RGB 图像失败", converted);
      }
      const auto publish_started_at = SteadyClock::now();
      publisher_->publish(video_pipeline::make_rgb_image(*rgb_frame_, header->second));
      stats.publish_time += SteadyClock::now() - publish_started_at;
      input_headers_.erase(header);
      ++stats.published_frames;
    }
  }

  void finish_stream()
  {
    if (decoding_started_) {
      DecodeStageStats stats;
      const auto decode_started_at = SteadyClock::now();
      const int send_result = decoder_strategy_->send_packet(nullptr);
      stats.decode_time += SteadyClock::now() - decode_started_at;
      if (send_result < 0) {
        throw ffmpeg_error("排空 H.264 解码器失败", send_result);
      }
      const int receive_result = publish_frames(stats);
      if (receive_result != AVERROR_EOF) {
        throw ffmpeg_error("读取剩余解码帧失败", receive_result);
      }
      if (!input_headers_.empty()) {
        throw std::runtime_error("解码结束时仍有数据包未输出图像");
      }
      accumulate_stage_stats(stats);
    }
    if (!publisher_->wait_for_all_acked(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "等待解码图像传输确认超时");
    }
    RCLCPP_INFO_STREAM(
      get_logger(), "H.264 解码结束，共发布 " << total_stage_stats_.published_frames << " 帧");
    rclcpp::shutdown();
  }

  std::unique_ptr<video_pipeline::H264DecoderStrategy> decoder_strategy_;
  video_pipeline::H264AnnexBParser bitstream_parser_;
  video_pipeline::ImageConverter converter_;
  std::unique_ptr<AVPacket, PacketDeleter> packet_;
  std::unique_ptr<AVFrame, FrameDeleter> decoded_frame_;
  std::unique_ptr<AVFrame, FrameDeleter> rgb_frame_;
  std::map<std::int64_t, std_msgs::msg::Header> input_headers_;
  bool decoding_started_ = false;
  const SteadyClock::time_point started_at_ = SteadyClock::now();
  std::mutex stage_stats_mutex_;
  DecodeStageStats total_stage_stats_;
  rclcpp::Publisher<Image>::SharedPtr publisher_;
  rclcpp::Subscription<H264Packet>::SharedPtr subscription_;
  rclcpp::Publisher<H264QueueFeedback>::SharedPtr feedback_publisher_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ready_publisher_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
  std::unique_ptr<H264PacketQueue> decode_queue_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::exception_ptr decode_error_;
  std::thread decode_thread_;
};

}  // 匿名命名空间

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<H264DecoderNode>();
    rclcpp::spin(node);
    node->check_decode_result();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("h264_decoder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
