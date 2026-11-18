#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

extern "C" {
#include <libavutil/imgutils.h>
}

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "video_pipeline/bitrate_controller.h"
#include "video_pipeline/ffmpeg_resources.h"
#include "video_pipeline/h264_packet_queue.h"
#include "video_pipeline/image_converter.h"
#include "video_pipeline/video_encoder.h"
#include "video_pipeline_msgs/msg/h264_packet.hpp"
#include "video_pipeline_msgs/msg/h264_queue_feedback.hpp"

namespace
{

using sensor_msgs::msg::Image;
using std_msgs::msg::UInt64;
using video_pipeline::BitrateChange;
using video_pipeline::BitrateController;
using video_pipeline::BitrateMetrics;
using video_pipeline::ffmpeg_error;
using video_pipeline::FrameDeleter;
using video_pipeline::H264PacketQueue;
using video_pipeline::PacketDeleter;
using video_pipeline_msgs::msg::H264Packet;
using video_pipeline_msgs::msg::H264QueueFeedback;

constexpr std::chrono::seconds kQueueFeedbackTimeout{1};
constexpr std::chrono::milliseconds kSamplePeriod{200};
constexpr std::chrono::seconds kRateWindow{1};

using SteadyClock = std::chrono::steady_clock;

struct EncoderSample
{
  SteadyClock::time_point sampled_at;
  video_pipeline::H264PacketQueueStats send_queue;
  std::int64_t input_frames;
  std::uint64_t encoded_payload_bytes;
  std::uint64_t published_packets;
  std::uint64_t published_payload_bytes;
  H264QueueFeedback::ConstSharedPtr decode_feedback;
  SteadyClock::time_point feedback_received_at;
};

class H264EncoderNode : public rclcpp::Node
{
public:
  H264EncoderNode()
  : Node("h264_encoder"), rgb_frame_(av_frame_alloc()), yuv_frame_(av_frame_alloc()),
    packet_(av_packet_alloc())
  {
    if (!rgb_frame_ || !yuv_frame_ || !packet_) {
      throw std::runtime_error("分配编码数据结构失败");
    }

    rcl_interfaces::msg::ParameterDescriptor startup_parameter;
    startup_parameter.read_only = true;
    const auto fps = declare_parameter<std::int64_t>("fps", 30, startup_parameter);
    encoder_config_.bit_rate = declare_parameter<std::int64_t>("bit_rate", 2000000, startup_parameter);
    if (fps <= 0 || fps > std::numeric_limits<int>::max() || encoder_config_.bit_rate <= 0) {
      throw std::invalid_argument("fps 必须是有效的正整数，bit_rate 必须大于 0");
    }
    encoder_config_.frames_per_second = static_cast<int>(fps);
    encoder_config_.gop_size = encoder_config_.frames_per_second;
    encoder_config_.encoder_name = "libx264";
    encoder_config_.tune = "zerolatency";
    if (declare_parameter<bool>("adaptive_bit_rate", false, startup_parameter)) {
      bitrate_controller_ = std::make_unique<BitrateController>(encoder_config_.bit_rate);
      encoder_config_.vbv_buffer_size =
        static_cast<int>(std::max<std::int64_t>(1000, encoder_config_.bit_rate / 2));
      encoder_config_.bit_rate = bitrate_controller_->bit_rate();
    }

    const auto max_packets = declare_parameter<std::int64_t>("queue_max_packets", 120, startup_parameter);
    const auto max_bytes = declare_parameter<std::int64_t>("queue_max_bytes", 8 * 1024 * 1024, startup_parameter);
    send_queue_ = std::make_unique<H264PacketQueue>("H.264 发送队列", max_packets, max_bytes);

    publisher_ = create_publisher<H264Packet>("video/h264", rclcpp::QoS(30));
    image_subscription_ = create_subscription<Image>(
      "video/image_raw", rclcpp::QoS(30),
      [this](Image::ConstSharedPtr image) {encode_image(*image);});
    end_subscription_ = create_subscription<UInt64>(
      "video/end_of_stream", rclcpp::QoS(1),
      [this](UInt64::ConstSharedPtr stream_end) {on_stream_end(stream_end->data);});
    feedback_subscription_ = create_subscription<H264QueueFeedback>(
      "video/queue_feedback", rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](H264QueueFeedback::ConstSharedPtr feedback) {
        latest_decode_feedback_ = std::move(feedback);
        decode_feedback_received_at_ = std::chrono::steady_clock::now();
      });
    stats_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {log_queue_stats();});
    if (bitrate_controller_) {
      bitrate_timer_ = create_wall_timer(kSamplePeriod, [this]() {update_bit_rate();});
      RCLCPP_INFO_STREAM(
        get_logger(), "H.264 自适应码率启动，初始目标 " << encoder_config_.bit_rate << " bit/s");
    }

    RCLCPP_INFO_STREAM(
      get_logger(), "等待 " << image_subscription_->get_topic_name()
                             << "，H.264 输出到 " << publisher_->get_topic_name());
    send_thread_ = std::thread([this]() {
      try {
        send_queued_packets();
      } catch (...) {
        if (rclcpp::ok()) {
          send_error_ = std::current_exception();
          rclcpp::shutdown();
        }
      }
    });
  }

  ~H264EncoderNode() override
  {
    stop_send_thread();
  }

  void check_send_result()
  {
    stop_send_thread();
    if (send_error_) {
      std::rethrow_exception(send_error_);
    }
  }

private:
  EncoderSample sample_encoder()
  {
    // 锁内同步采集出队和发布计数，发布本身由发送线程在锁外执行。
    std::lock_guard<std::mutex> lock(publish_stats_mutex_);
    const auto queue_stats = send_queue_->stats();
    return {
      SteadyClock::now(), queue_stats, next_pts_, total_encoded_payload_bytes_,
      total_published_packets_, total_published_payload_bytes_, latest_decode_feedback_,
      decode_feedback_received_at_
    };
  }

  void log_queue_stats()
  {
    const auto sample = sample_encoder();
    const auto & queue_stats = sample.send_queue;
    RCLCPP_INFO_STREAM(get_logger(), "H.264 发送队列：" << queue_stats);
    // 当前 PTS 从零开始，每成功送入一帧加一，也就是累计输入帧数。
    RCLCPP_INFO_STREAM(
      get_logger(), "H.264 编码端累计：输入 " << sample.input_frames << " 帧，入队 " <<
        queue_stats.total_enqueued_packets << " 包，出队 " << queue_stats.total_dequeued_packets <<
        " 包，发布 " << sample.published_packets << " 包，数据 " <<
        sample.published_payload_bytes << " 字节");
    if (!sample.decode_feedback) {
      RCLCPP_INFO(get_logger(), "H.264 解码队列反馈：尚未收到");
      return;
    }

    // 用编码节点的单调时钟计算接收间隔，允许两个节点跨主机运行。
    const auto age = sample.sampled_at - sample.feedback_received_at;
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(age).count();
    if (age >= kQueueFeedbackTimeout) {
      RCLCPP_INFO_STREAM(get_logger(), "H.264 解码队列反馈：已过期，接收于 " << age_ms << " ms 前");
      return;
    }

    const auto & feedback = *sample.decode_feedback;
    RCLCPP_INFO_STREAM(
      get_logger(), "H.264 解码队列反馈：有效，" << feedback.packet_count << "/" <<
        feedback.max_packets << " 包，" << feedback.byte_count << "/" << feedback.max_bytes <<
        " 字节，最老包等待 " << feedback.oldest_wait_ms << " ms，接收于 " << age_ms << " ms 前");
  }

  void read_decode_metrics(
    const EncoderSample & before, const EncoderSample & current, BitrateMetrics & metrics)
  {
    if (!before.decode_feedback || !current.decode_feedback ||
      before.sampled_at - before.feedback_received_at >= kQueueFeedbackTimeout ||
      current.sampled_at - current.feedback_received_at >= kQueueFeedbackTimeout)
    {
      return;
    }

    // 远端计数只用解码端自己的单调时钟计算，反馈丢失时也按实际时间差计算。
    const auto & first = *before.decode_feedback;
    const auto & last = *current.decode_feedback;
    if (last.sample_elapsed_us <= first.sample_elapsed_us ||
      last.total_enqueued_packets < first.total_enqueued_packets ||
      last.total_dequeued_packets < first.total_dequeued_packets ||
      last.total_decode_time_us < first.total_decode_time_us ||
      last.total_convert_time_us < first.total_convert_time_us ||
      last.total_publish_time_us < first.total_publish_time_us)
    {
      return;
    }
    const auto elapsed_us = last.sample_elapsed_us - first.sample_elapsed_us;
    if (elapsed_us < 500000 || elapsed_us > 2000000) {
      return;
    }
    const auto decode_us = last.total_decode_time_us - first.total_decode_time_us;
    const auto convert_us = last.total_convert_time_us - first.total_convert_time_us;
    const auto publish_us = last.total_publish_time_us - first.total_publish_time_us;
    const double seconds = elapsed_us / 1000000.0;
    metrics.decode_wait_ms = last.oldest_wait_ms;
    metrics.decode_enqueue_packets_per_second =
      (last.total_enqueued_packets - first.total_enqueued_packets) / seconds;
    metrics.decode_dequeue_packets_per_second =
      (last.total_dequeued_packets - first.total_dequeued_packets) / seconds;
    metrics.decode_busy_fraction = (static_cast<double>(decode_us) + convert_us + publish_us) /
      elapsed_us;
    metrics.feedback_valid = true;
  }

  void update_bit_rate()
  {
    if (next_pts_ == 0 || end_received_) {
      return;
    }
    const auto current = sample_encoder();
    samples_.push_back(current);
    while (samples_.size() > 1 && current.sampled_at - samples_[1].sampled_at >= kRateWindow) {
      samples_.pop_front();
    }
    const auto & before = samples_.front();
    const auto elapsed = current.sampled_at - before.sampled_at;
    if (elapsed < kRateWindow || elapsed >= 2 * kRateWindow) {
      return;
    }
    const double seconds = std::chrono::duration<double>(elapsed).count();
    BitrateMetrics metrics;
    metrics.sampled_at = current.sampled_at;
    metrics.send_wait_ms = static_cast<std::uint64_t>(current.send_queue.oldest_wait.count());
    metrics.encoded_packets_per_second =
      (current.send_queue.total_enqueued_packets - before.send_queue.total_enqueued_packets) /
      seconds;
    metrics.published_packets_per_second =
      (current.published_packets - before.published_packets) / seconds;
    metrics.encoded_bits_per_second =
      (current.encoded_payload_bytes - before.encoded_payload_bytes) * 8.0 / seconds;
    metrics.published_bits_per_second =
      (current.published_payload_bytes - before.published_payload_bytes) * 8.0 / seconds;
    read_decode_metrics(before, current, metrics);

    const auto decision = bitrate_controller_->update(metrics);
    if (decision.change == BitrateChange::Hold) {
      return;
    }
    // 定时器和图像回调都由节点执行器串行调用，更新会在下一帧编码时生效。
    const auto previous_rate = encoder_config_.bit_rate;
    const int result = encoder_.set_bit_rate(decision.bit_rate);
    if (result < 0) {
      throw ffmpeg_error("更新 H.264 目标码率失败", result);
    }
    encoder_config_.bit_rate = decision.bit_rate;

    const char * reason = "稳定增速";
    if (decision.change == BitrateChange::SendCongestion) {
      reason = "发送拥塞";
    } else if (decision.change == BitrateChange::SlowStart) {
      reason = "慢启动";
    }
    RCLCPP_INFO_STREAM(
      get_logger(), "H.264 码率调整：" << reason << "，目标 " << previous_rate << " -> " <<
        decision.bit_rate << " bit/s，ssthresh " << decision.slow_start_threshold <<
        " bit/s，窗口编码 " << static_cast<std::int64_t>(metrics.encoded_bits_per_second) <<
        " bit/s，发布 " << static_cast<std::int64_t>(metrics.published_bits_per_second) << " bit/s");
  }

  void stop_send_thread()
  {
    if (send_thread_.joinable()) {
      send_queue_->cancel();
      send_thread_.join();
    }
  }

  void send_queued_packets()
  {
    H264PacketQueue::Packet packet;
    while (rclcpp::ok()) {
      const auto result = send_queue_->wait_pop(packet);
      if (result == H264PacketQueue::PopResult::Cancelled || !rclcpp::ok()) {
        return;
      }
      if (result == H264PacketQueue::PopResult::Finished) {
        break;
      }
      publisher_->publish(*packet);
      {
        std::lock_guard<std::mutex> lock(publish_stats_mutex_);
        ++total_published_packets_;
        total_published_payload_bytes_ += packet->data.size();
      }
    }
    if (!rclcpp::ok()) {
      return;
    }

    // 发送线程取完所有编码包后，才在同一话题发送结束标记。
    H264Packet stream_end;
    stream_end.end_of_stream = true;
    publisher_->publish(stream_end);
    if (!publisher_->wait_for_all_acked(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "等待 H.264 传输确认超时");
    }
    RCLCPP_INFO_STREAM(
      get_logger(), "H.264 发送结束，共发布 " << total_published_packets_ << " 包，数据 " <<
        total_published_payload_bytes_ << " 字节");
    rclcpp::shutdown();
  }

  void open_encoder(const Image & image)
  {
    const int result = av_image_check_size(image.width, image.height, 0, nullptr);
    if (result < 0) {
      throw ffmpeg_error("检查输入图像尺寸失败", result);
    }
    if (image.width % 2 != 0 || image.height % 2 != 0) {
      throw std::invalid_argument("YUV420P 编码要求图像宽高均为偶数");
    }

    encoder_config_.width = static_cast<int>(image.width);
    encoder_config_.height = static_cast<int>(image.height);
    rgb_frame_->format = AV_PIX_FMT_RGB24;
    rgb_frame_->width = encoder_config_.width;
    rgb_frame_->height = encoder_config_.height;
    const int buffer_result = av_frame_get_buffer(rgb_frame_.get(), 32);
    if (buffer_result < 0) {
      throw ffmpeg_error("分配 RGB 输入缓冲失败", buffer_result);
    }
    const int open_result = encoder_.open(encoder_config_);
    if (open_result < 0) {
      throw ffmpeg_error("打开 H.264 编码器失败", open_result);
    }
    RCLCPP_INFO_STREAM(
      get_logger(), "开始编码 " << image.width << "x" << image.height << "，"
                                 << encoder_config_.frames_per_second << " fps");
  }

  void encode_image(const Image & image)
  {
    const std::uint64_t row_bytes = static_cast<std::uint64_t>(image.width) * 3;
    const std::uint64_t data_bytes = static_cast<std::uint64_t>(image.step) * image.height;
    if (image.encoding != sensor_msgs::image_encodings::RGB8 || image.width == 0 ||
      image.height == 0 || image.step < row_bytes || image.data.size() != data_bytes)
    {
      throw std::invalid_argument("输入必须是宽高、行跨度和数据长度有效的 rgb8 图像");
    }
    if (next_pts_ == 0) {
      open_encoder(image);
    }
    if (image.width != static_cast<std::uint32_t>(encoder_config_.width) ||
      image.height != static_cast<std::uint32_t>(encoder_config_.height))
    {
      throw std::invalid_argument("同一视频流内的图像尺寸必须一致");
    }

    // ROS 消息和 FFmpeg 帧的行跨度可能不同，只逐行复制有效像素。
    for (std::uint32_t row = 0; row < image.height; ++row) {
      const auto * source = image.data.data() + static_cast<std::size_t>(row) * image.step;
      auto * destination = rgb_frame_->data[0] +
        static_cast<std::size_t>(row) * rgb_frame_->linesize[0];
      std::memcpy(destination, source, static_cast<std::size_t>(row_bytes));
    }
    const int result = converter_.convert(rgb_frame_.get(), encoder_config_.pixel_format, yuv_frame_.get());
    if (result < 0) {
      throw ffmpeg_error("转换 YUV420P 图像失败", result);
    }

    // 编码器可能延迟输出，按 PTS 保留对应输入帧的 ROS 时间戳和坐标系。
    yuv_frame_->pts = next_pts_;
    const int send_result = encoder_.send_frame(yuv_frame_.get());
    if (send_result < 0) {
      throw ffmpeg_error("送入待编码视频帧失败", send_result);
    }
    input_headers_.emplace(next_pts_, image.header);
    ++next_pts_;
    const int receive_result = queue_encoded_packets();
    if (receive_result != AVERROR(EAGAIN)) {
      throw ffmpeg_error("读取 H.264 编码包失败", receive_result);
    }
    finish_if_ready();
  }

  int queue_encoded_packets()
  {
    const AVRational time_base = encoder_.time_base();
    while (true) {
      const int result = encoder_.receive_packet(packet_.get());
      if (result < 0) {
        return result;
      }

      const auto header = input_headers_.find(packet_->pts);
      if (header == input_headers_.end()) {
        throw std::runtime_error("找不到编码包对应的输入帧");
      }
      auto message = std::make_shared<H264Packet>();
      message->header = header->second;
      message->pts = packet_->pts;
      message->dts = packet_->dts;
      message->time_base_num = time_base.num;
      message->time_base_den = time_base.den;
      message->key_frame = (packet_->flags & AV_PKT_FLAG_KEY) != 0;
      message->data.assign(packet_->data, packet_->data + packet_->size);
      const auto payload_bytes = message->data.size();
      send_queue_->push(std::move(message));
      total_encoded_payload_bytes_ += payload_bytes;
      input_headers_.erase(header);
    }
  }

  void on_stream_end(std::uint64_t frame_count)
  {
    if (end_received_) {
      return;
    }
    end_received_ = true;
    expected_frames_ = frame_count;
    if (static_cast<std::uint64_t>(next_pts_) != expected_frames_) {
      // 两个输入话题独立传输，结束通知可能先于最后几帧进入回调。
      end_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
        throw std::runtime_error(
                "结束通知与收到的帧数不符，请先启动编码节点再启动视频发布节点");
      });
    }
    finish_if_ready();
  }

  void finish_if_ready()
  {
    if (!end_received_ || static_cast<std::uint64_t>(next_pts_) != expected_frames_) {
      return;
    }
    if (end_timer_) {
      end_timer_->cancel();
    }
    if (next_pts_ > 0) {
      const int result = encoder_.send_frame(nullptr);
      if (result < 0) {
        throw ffmpeg_error("排空 H.264 编码器失败", result);
      }
      const int receive_result = queue_encoded_packets();
      if (receive_result != AVERROR_EOF) {
        throw ffmpeg_error("读取剩余 H.264 编码包失败", receive_result);
      }
      if (!input_headers_.empty()) {
        throw std::runtime_error("编码结束时仍有输入帧未输出");
      }
    }

    RCLCPP_INFO_STREAM(get_logger(), "H.264 编码结束，共处理 " << next_pts_ << " 帧");
    send_queue_->close();
  }

  video_pipeline::VideoEncoder encoder_;
  video_pipeline::VideoEncoderConfig encoder_config_;
  std::unique_ptr<BitrateController> bitrate_controller_;
  std::deque<EncoderSample> samples_;
  video_pipeline::ImageConverter converter_;
  std::unique_ptr<AVFrame, FrameDeleter> rgb_frame_;
  std::unique_ptr<AVFrame, FrameDeleter> yuv_frame_;
  std::unique_ptr<AVPacket, PacketDeleter> packet_;
  std::map<std::int64_t, std_msgs::msg::Header> input_headers_;
  std::int64_t next_pts_ = 0;
  std::uint64_t expected_frames_ = 0;
  bool end_received_ = false;
  rclcpp::Publisher<H264Packet>::SharedPtr publisher_;
  rclcpp::Subscription<Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<UInt64>::SharedPtr end_subscription_;
  rclcpp::Subscription<H264QueueFeedback>::SharedPtr feedback_subscription_;
  H264QueueFeedback::ConstSharedPtr latest_decode_feedback_;
  std::chrono::steady_clock::time_point decode_feedback_received_at_;
  rclcpp::TimerBase::SharedPtr end_timer_;
  std::unique_ptr<H264PacketQueue> send_queue_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr bitrate_timer_;
  std::uint64_t total_encoded_payload_bytes_ = 0;
  std::mutex publish_stats_mutex_;
  std::uint64_t total_published_packets_ = 0;
  std::uint64_t total_published_payload_bytes_ = 0;
  std::exception_ptr send_error_;
  std::thread send_thread_;
};

}  // 匿名命名空间

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<H264EncoderNode>();
    rclcpp::spin(node);
    node->check_send_result();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("h264_encoder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
