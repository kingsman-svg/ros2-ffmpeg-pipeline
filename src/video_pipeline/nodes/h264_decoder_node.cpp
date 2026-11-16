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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/header.hpp"

#include "video_pipeline/ffmpeg_resources.h"
#include "video_pipeline/image_converter.h"
#include "video_pipeline/media_decoder.h"
#include "video_pipeline/rgb_image_message.h"
#include "video_pipeline_msgs/msg/h264_packet.hpp"

namespace
{

using sensor_msgs::msg::Image;
using video_pipeline::ffmpeg_error;
using video_pipeline::FrameDeleter;
using video_pipeline::PacketDeleter;
using video_pipeline_msgs::msg::H264Packet;

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

    // Annex B 流中的 SPS 和 PPS 会提供宽高等参数，初始化时只需指定编码格式。
    auto free_parameters = [](AVCodecParameters * parameters) {
        avcodec_parameters_free(&parameters);
      };
    std::unique_ptr<AVCodecParameters, decltype(free_parameters)> parameters(
      avcodec_parameters_alloc(), free_parameters);
    if (!parameters) {
      throw std::runtime_error("分配解码参数失败");
    }
    parameters->codec_type = AVMEDIA_TYPE_VIDEO;
    parameters->codec_id = AV_CODEC_ID_H264;
    const int result = decoder_.open(parameters.get());
    if (result < 0) {
      throw ffmpeg_error("打开 H.264 解码器失败", result);
    }

    publisher_ = create_publisher<Image>("video/decoded_image", rclcpp::QoS(30));
    subscription_ = create_subscription<H264Packet>(
      "video/h264", rclcpp::QoS(30),
      [this](H264Packet::ConstSharedPtr message) {decode_packet(*message);});
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
    RCLCPP_INFO_STREAM(
      get_logger(), "等待 " << subscription_->get_topic_name()
                             << "，解码图像输出到 " << publisher_->get_topic_name());
  }

private:
  void decode_packet(const H264Packet & message)
  {
    if (message.end_of_stream) {
      finish_stream();
      return;
    }
    // 中途加入时从关键帧开始，避免把缺少参考帧的包送给解码器。
    if (waiting_for_key_frame_ && !message.key_frame) {
      return;
    }
    const auto max_packet_size = static_cast<std::size_t>(
      std::numeric_limits<int>::max() - AV_INPUT_BUFFER_PADDING_SIZE);
    if (message.data.empty() || message.data.size() > max_packet_size ||
      message.pts == AV_NOPTS_VALUE)
    {
      throw std::invalid_argument("H.264 数据包大小或 PTS 无效");
    }

    // av_new_packet 会在数据末尾分配 FFmpeg 解码所需的零填充。
    av_packet_unref(packet_.get());
    const int allocation_result = av_new_packet(packet_.get(), static_cast<int>(message.data.size()));
    if (allocation_result < 0) {
      throw ffmpeg_error("分配 H.264 数据包失败", allocation_result);
    }
    std::memcpy(packet_->data, message.data.data(), message.data.size());
    packet_->pts = message.pts;
    packet_->dts = message.dts;
    if (message.key_frame) {
      packet_->flags |= AV_PKT_FLAG_KEY;
    }

    const int send_result = decoder_.send_packet(packet_.get());
    if (send_result < 0) {
      throw ffmpeg_error("送入 H.264 数据包失败", send_result);
    }
    waiting_for_key_frame_ = false;
    // 解码可能延迟或重排，按 PTS 找回每帧自己的 ROS 头部。
    if (!input_headers_.emplace(message.pts, message.header).second) {
      throw std::invalid_argument("收到重复的 H.264 PTS");
    }
    const int receive_result = publish_frames();
    if (receive_result != AVERROR(EAGAIN)) {
      throw ffmpeg_error("读取解码帧失败", receive_result);
    }
  }

  int publish_frames()
  {
    while (true) {
      const int result = decoder_.receive_frame(decoded_frame_.get());
      if (result < 0) {
        return result;
      }
      const auto header = input_headers_.find(decoded_frame_->pts);
      if (header == input_headers_.end()) {
        throw std::runtime_error("找不到解码帧对应的 ROS 头部");
      }
      const int converted = converter_.convert(decoded_frame_.get(), AV_PIX_FMT_RGB24, rgb_frame_.get());
      if (converted < 0) {
        throw ffmpeg_error("转换 RGB 图像失败", converted);
      }
      publisher_->publish(video_pipeline::make_rgb_image(*rgb_frame_, header->second));
      input_headers_.erase(header);
      ++published_frames_;
    }
  }

  void finish_stream()
  {
    if (!waiting_for_key_frame_) {
      const int send_result = decoder_.send_packet(nullptr);
      if (send_result < 0) {
        throw ffmpeg_error("排空 H.264 解码器失败", send_result);
      }
      const int receive_result = publish_frames();
      if (receive_result != AVERROR_EOF) {
        throw ffmpeg_error("读取剩余解码帧失败", receive_result);
      }
      if (!input_headers_.empty()) {
        throw std::runtime_error("解码结束时仍有数据包未输出图像");
      }
    }
    if (!publisher_->wait_for_all_acked(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "等待解码图像传输确认超时");
    }
    RCLCPP_INFO_STREAM(get_logger(), "H.264 解码结束，共发布 " << published_frames_ << " 帧");
    rclcpp::shutdown();
  }

  video_pipeline::MediaDecoder decoder_;
  video_pipeline::ImageConverter converter_;
  std::unique_ptr<AVPacket, PacketDeleter> packet_;
  std::unique_ptr<AVFrame, FrameDeleter> decoded_frame_;
  std::unique_ptr<AVFrame, FrameDeleter> rgb_frame_;
  std::map<std::int64_t, std_msgs::msg::Header> input_headers_;
  bool waiting_for_key_frame_ = true;
  std::uint64_t published_frames_ = 0;
  rclcpp::Publisher<Image>::SharedPtr publisher_;
  rclcpp::Subscription<H264Packet>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ready_publisher_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
};

}  // 匿名命名空间

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<H264DecoderNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("h264_decoder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
