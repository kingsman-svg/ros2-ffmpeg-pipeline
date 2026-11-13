#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

extern "C" {
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
}

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "video_pipeline/image_converter.h"
#include "video_pipeline/media_decoder.h"
#include "video_pipeline/media_reader.h"

namespace
{

struct PacketDeleter
{
  void operator()(AVPacket * packet) const
  {
    av_packet_free(&packet);
  }
};

struct FrameDeleter
{
  void operator()(AVFrame * frame) const
  {
    av_frame_free(&frame);
  }
};

std::runtime_error ffmpeg_error(const std::string & action, int error)
{
  char message[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(error, message, sizeof(message));
  return std::runtime_error(action + "：" + message);
}

class VideoPipelineNode : public rclcpp::Node
{
public:
  VideoPipelineNode()
  : Node("video_pipeline"), packet_(av_packet_alloc()),
    decoded_frame_(av_frame_alloc()), rgb_frame_(av_frame_alloc())
  {
    rcl_interfaces::msg::ParameterDescriptor startup_parameter;
    startup_parameter.read_only = true;
    const auto input_path = declare_parameter<std::string>("input_path", "", startup_parameter);
    const double fps = declare_parameter<double>("fps", 30.0, startup_parameter);
    if (input_path.empty()) {
      throw std::invalid_argument("请通过 input_path 参数指定视频文件路径");
    }
    // 定时器使用整数纳秒，先检查周期能否用该类型表示。
    const double period_ns = 1e9 / fps;
    using Nanoseconds = std::chrono::nanoseconds;
    if (!(period_ns >= 1.0 && period_ns < static_cast<double>(Nanoseconds::max().count()))) {
      throw std::invalid_argument("fps 必须为正数，且对应的定时器周期必须有效");
    }
    if (!packet_ || !decoded_frame_ || !rgb_frame_) {
      throw std::runtime_error("分配视频数据结构失败");
    }

    int result = reader_.open(input_path, AVMEDIA_TYPE_VIDEO);
    if (result < 0) {
      throw ffmpeg_error("打开视频失败 " + input_path, result);
    }
    result = decoder_.open(reader_.codec_parameters());
    if (result < 0) {
      throw ffmpeg_error("初始化视频解码器失败", result);
    }

    publisher_ = create_publisher<sensor_msgs::msg::Image>("video/image_raw", rclcpp::SensorDataQoS());
    const Nanoseconds period(static_cast<Nanoseconds::rep>(period_ns));
    timer_ = create_wall_timer(period, [this]() {publish_next_frame();});
    RCLCPP_INFO_STREAM(
      get_logger(), "发布视频 " << input_path << " 到 " << publisher_->get_topic_name()
                                 << "，设定帧率 " << fps << " fps");
  }

private:
  bool read_next_frame()
  {
    while (rclcpp::ok()) {
      // 先取解码器中已有的帧，需要更多输入时再读下一个 Packet。
      int result = decoder_.receive_frame(decoded_frame_.get());
      if (result == 0) {
        return true;
      }
      if (result == AVERROR_EOF) {
        return false;
      }
      if (result != AVERROR(EAGAIN)) {
        throw ffmpeg_error("解码视频帧失败", result);
      }

      result = reader_.read_packet(packet_.get());
      if (result < 0 && result != AVERROR_EOF) {
        throw ffmpeg_error("读取视频数据失败", result);
      }
      // 文件结束时送入空 Packet，继续取出解码器缓冲的帧。
      result = decoder_.send_packet(result == AVERROR_EOF ? nullptr : packet_.get());
      if (result < 0) {
        throw ffmpeg_error("向解码器送入视频数据失败", result);
      }
    }
    return false;
  }

  void publish_next_frame()
  {
    if (!read_next_frame()) {
      if (rclcpp::ok()) {
        RCLCPP_INFO_STREAM(get_logger(), "视频发布结束，共发布 " << published_frames_ << " 帧");
        rclcpp::shutdown();
      }
      return;
    }

    const auto stamp = now();
    const int result = converter_.convert(decoded_frame_.get(), AV_PIX_FMT_RGB24, rgb_frame_.get());
    if (result < 0) {
      throw ffmpeg_error("转换 RGB 图像失败", result);
    }
    const int width = rgb_frame_->width;
    const int height = rgb_frame_->height;
    const int image_size = av_image_get_buffer_size(AV_PIX_FMT_RGB24, width, height, 1);
    if (image_size < 0) {
      throw ffmpeg_error("计算 RGB 图像大小失败", image_size);
    }

    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = "video_optical_frame";
    image.width = static_cast<std::uint32_t>(width);
    image.height = static_cast<std::uint32_t>(height);
    image.encoding = sensor_msgs::image_encodings::RGB8;
    image.is_bigendian = 0;
    image.step = image.width * 3;
    image.data.resize(static_cast<std::size_t>(image_size));

    // 按源帧的行跨度读取，以 1 字节对齐写入，去掉行末填充。
    const std::uint8_t * planes[4] = {rgb_frame_->data[0], nullptr, nullptr, nullptr};
    const int copied = av_image_copy_to_buffer(
      image.data.data(), image_size, planes, rgb_frame_->linesize, AV_PIX_FMT_RGB24,
      width, height, 1);
    if (copied < 0) {
      throw ffmpeg_error("复制 RGB 图像失败", copied);
    }
    publisher_->publish(image);
    ++published_frames_;
  }

  video_pipeline::MediaReader reader_;
  video_pipeline::MediaDecoder decoder_;
  video_pipeline::ImageConverter converter_;
  std::unique_ptr<AVPacket, PacketDeleter> packet_;
  std::unique_ptr<AVFrame, FrameDeleter> decoded_frame_;
  std::unique_ptr<AVFrame, FrameDeleter> rgb_frame_;
  std::uint64_t published_frames_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // 匿名命名空间

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoPipelineNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("video_pipeline"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
