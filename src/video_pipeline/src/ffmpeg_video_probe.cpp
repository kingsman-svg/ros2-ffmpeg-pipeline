#include <cerrno>
#include <cstdint>
#include <iostream>
#include <memory>

extern "C" {
#include <libavutil/error.h>
}

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

void print_error(const char * action, int error)
{
  char message[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(error, message, sizeof(message));
  std::cerr << action << "：" << message << std::endl;
}

// 取出当前所有可用帧，直到需要更多数据或解码结束。
int receive_frames(
  video_pipeline::MediaDecoder & decoder,
  video_pipeline::ImageConverter & converter,
  AVFrame * frame, AVFrame * rgb_frame, std::uint64_t & frame_count)
{
  while (true) {
    int result = decoder.receive_frame(frame);
    if (result < 0) {
      return result;
    }

    result = converter.convert(frame, AV_PIX_FMT_RGB24, rgb_frame);
    if (result < 0) {
      return result;
    }

    ++frame_count;
    if (frame_count == 1) {
      std::cout << "首帧 RGB24：" << rgb_frame->width << "x" << rgb_frame->height
                << "，每行字节数：" << rgb_frame->linesize[0] << std::endl;
    }
  }
}

// 解码器暂时不接收输入时，先取走已解出的帧再重试。
int decode_packet(
  video_pipeline::MediaDecoder & decoder,
  video_pipeline::ImageConverter & converter,
  const AVPacket * packet, AVFrame * frame, AVFrame * rgb_frame,
  std::uint64_t & frame_count)
{
  int result = decoder.send_packet(packet);
  if (result == AVERROR(EAGAIN)) {
    result = receive_frames(decoder, converter, frame, rgb_frame, frame_count);
    if (result != AVERROR(EAGAIN)) {
      return result;
    }
    result = decoder.send_packet(packet);
    if (result == AVERROR(EAGAIN)) {
      return AVERROR_BUG;
    }
  }
  if (result < 0) {
    return result;
  }
  return receive_frames(decoder, converter, frame, rgb_frame, frame_count);
}

}  // 匿名命名空间

int main(int argc, char * argv[])
{
  if (argc != 2) {
    std::cerr << "用法：ffmpeg_video_probe <视频文件路径>" << std::endl;
    return 1;
  }

  video_pipeline::MediaReader reader;
  int result = reader.open(argv[1], AVMEDIA_TYPE_VIDEO);
  if (result < 0) {
    print_error("打开视频失败", result);
    return 1;
  }

  const AVCodecParameters * parameters = reader.codec_parameters();
  const AVRational time_base = reader.time_base();
  std::cout << "视频编码：" << avcodec_get_name(parameters->codec_id) << std::endl;
  std::cout << "视频尺寸：" << parameters->width << "x" << parameters->height << std::endl;
  std::cout << "时间基：" << time_base.num << "/" << time_base.den << std::endl;

  video_pipeline::MediaDecoder decoder;
  result = decoder.open(parameters);
  if (result < 0) {
    print_error("初始化解码器失败", result);
    return 1;
  }

  std::unique_ptr<AVPacket, PacketDeleter> packet(av_packet_alloc());
  std::unique_ptr<AVFrame, FrameDeleter> frame(av_frame_alloc());
  std::unique_ptr<AVFrame, FrameDeleter> rgb_frame(av_frame_alloc());
  if (!packet || !frame || !rgb_frame) {
    std::cerr << "分配视频数据结构失败" << std::endl;
    return 1;
  }

  video_pipeline::ImageConverter converter;
  std::uint64_t packet_count = 0;
  std::uint64_t frame_count = 0;
  while (true) {
    result = reader.read_packet(packet.get());
    if (result == AVERROR_EOF) {
      break;
    }
    if (result < 0) {
      print_error("读取视频数据失败", result);
      return 1;
    }

    ++packet_count;
    result = decode_packet(decoder, converter, packet.get(), frame.get(), rgb_frame.get(), frame_count);
    if (result != AVERROR(EAGAIN)) {
      print_error("解码视频失败", result);
      return 1;
    }
  }

  // 文件结束后继续取出解码器内部缓冲的帧。
  result = decode_packet(decoder, converter, nullptr, frame.get(), rgb_frame.get(), frame_count);
  if (result != AVERROR_EOF) {
    print_error("刷新解码器失败", result);
    return 1;
  }

  std::cout << "视频 Packet 数：" << packet_count << std::endl;
  std::cout << "解码帧数：" << frame_count << std::endl;
  return 0;
}
