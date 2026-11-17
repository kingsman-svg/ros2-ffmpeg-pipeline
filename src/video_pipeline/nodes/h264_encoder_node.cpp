#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <map>
#include <memory>
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
using video_pipeline::ffmpeg_error;
using video_pipeline::FrameDeleter;
using video_pipeline::H264PacketQueue;
using video_pipeline::PacketDeleter;
using video_pipeline_msgs::msg::H264Packet;
using video_pipeline_msgs::msg::H264QueueFeedback;

constexpr std::chrono::seconds kQueueFeedbackTimeout{1};

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
  void log_queue_stats()
  {
    RCLCPP_INFO_STREAM(get_logger(), "H.264 发送队列：" << send_queue_->stats());
    if (!latest_decode_feedback_) {
      RCLCPP_INFO(get_logger(), "H.264 解码队列反馈：尚未收到");
      return;
    }

    // 用编码节点的单调时钟计算接收间隔，允许两个节点跨主机运行。
    const auto age = std::chrono::steady_clock::now() - decode_feedback_received_at_;
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(age).count();
    if (age >= kQueueFeedbackTimeout) {
      RCLCPP_INFO_STREAM(get_logger(), "H.264 解码队列反馈：已过期，接收于 " << age_ms << " ms 前");
      return;
    }

    const auto & feedback = *latest_decode_feedback_;
    RCLCPP_INFO_STREAM(
      get_logger(), "H.264 解码队列反馈：有效，" << feedback.packet_count << "/" <<
        feedback.max_packets << " 包，" << feedback.byte_count << "/" << feedback.max_bytes <<
        " 字节，最老包等待 " << feedback.oldest_wait_ms << " ms，接收于 " << age_ms << " ms 前");
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
      send_queue_->push(std::move(message));
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
