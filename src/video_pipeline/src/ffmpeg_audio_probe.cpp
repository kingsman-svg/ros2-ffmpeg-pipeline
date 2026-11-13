#include <cerrno>
#include <cstdint>
#include <iostream>
#include <memory>

extern "C" {
#include <libavutil/channel_layout.h>
#include <libavutil/error.h>
}

#include "video_pipeline/audio_resampler.h"
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

int receive_frames(
  video_pipeline::MediaDecoder & decoder, video_pipeline::AudioResampler & resampler,
  AVFrame * frame, AVFrame * converted, bool & resampler_open,
  std::uint64_t & input_samples, std::uint64_t & output_samples)
{
  while (true) {
    int result = decoder.receive_frame(frame);
    if (result < 0) {
      return result;
    }

    if (!resampler_open) {
      std::int64_t layout = static_cast<std::int64_t>(frame->channel_layout);
      if (layout == 0 && (frame->channels == 1 || frame->channels == 2)) {
        // 缺少声道布局时，只对常见单声道和双声道使用默认布局。
        layout = av_get_default_channel_layout(frame->channels);
      }
      result = resampler.open(
        frame->sample_rate, static_cast<AVSampleFormat>(frame->format), layout,
        48000, AV_SAMPLE_FMT_S16, AV_CH_LAYOUT_STEREO);
      if (result < 0) {
        return result;
      }
      resampler_open = true;
      std::cout << "输入采样率：" << frame->sample_rate << std::endl;
      std::cout << "输入声道数：" << frame->channels << std::endl;
      std::cout << "输出格式：48000 Hz，双声道，S16" << std::endl;
    }

    result = resampler.convert(frame, converted);
    if (result < 0) {
      return result;
    }
    input_samples += frame->nb_samples;
    output_samples += result;
  }
}

int decode_packet(
  video_pipeline::MediaDecoder & decoder, video_pipeline::AudioResampler & resampler,
  const AVPacket * packet, AVFrame * frame, AVFrame * converted, bool & resampler_open,
  std::uint64_t & input_samples, std::uint64_t & output_samples)
{
  int result = decoder.send_packet(packet);
  if (result == AVERROR(EAGAIN)) {
    result = receive_frames(
      decoder, resampler, frame, converted, resampler_open, input_samples, output_samples);
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
  return receive_frames(
    decoder, resampler, frame, converted, resampler_open, input_samples, output_samples);
}

}  // 匿名命名空间

int main(int argc, char * argv[])
{
  if (argc != 2) {
    std::cerr << "用法：ffmpeg_audio_probe <音频文件路径>" << std::endl;
    return 1;
  }

  video_pipeline::MediaReader reader;
  int result = reader.open(argv[1], AVMEDIA_TYPE_AUDIO);
  if (result < 0) {
    print_error("打开音频失败", result);
    return 1;
  }

  video_pipeline::MediaDecoder decoder;
  result = decoder.open(reader.codec_parameters());
  if (result < 0) {
    print_error("初始化解码器失败", result);
    return 1;
  }

  std::unique_ptr<AVPacket, PacketDeleter> packet(av_packet_alloc());
  std::unique_ptr<AVFrame, FrameDeleter> frame(av_frame_alloc());
  std::unique_ptr<AVFrame, FrameDeleter> converted(av_frame_alloc());
  if (!packet || !frame || !converted) {
    std::cerr << "分配音频数据结构失败" << std::endl;
    return 1;
  }

  video_pipeline::AudioResampler resampler;
  bool resampler_open = false;
  std::uint64_t input_samples = 0;
  std::uint64_t output_samples = 0;
  while (true) {
    result = reader.read_packet(packet.get());
    if (result == AVERROR_EOF) {
      break;
    }
    if (result < 0) {
      print_error("读取音频数据失败", result);
      return 1;
    }

    result = decode_packet(
      decoder, resampler, packet.get(), frame.get(), converted.get(), resampler_open,
      input_samples, output_samples);
    if (result != AVERROR(EAGAIN)) {
      print_error("解码或重采样音频失败", result);
      return 1;
    }
  }

  result = decode_packet(
    decoder, resampler, nullptr, frame.get(), converted.get(), resampler_open,
    input_samples, output_samples);
  if (result != AVERROR_EOF) {
    print_error("刷新解码器失败", result);
    return 1;
  }
  if (!resampler_open) {
    std::cerr << "音频中没有可解码的采样" << std::endl;
    return 1;
  }

  // 解码结束后排出重采样器内部缓冲的采样。
  while (true) {
    result = resampler.convert(nullptr, converted.get());
    if (result < 0) {
      print_error("刷新重采样器失败", result);
      return 1;
    }
    if (result == 0) {
      break;
    }
    output_samples += result;
  }

  std::cout << "输入采样数（每声道）：" << input_samples << std::endl;
  std::cout << "输出采样数（每声道）：" << output_samples << std::endl;
  return 0;
}
