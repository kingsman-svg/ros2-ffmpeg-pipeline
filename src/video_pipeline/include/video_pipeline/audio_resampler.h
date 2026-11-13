#ifndef VIDEO_PIPELINE_AUDIO_RESAMPLER_H_
#define VIDEO_PIPELINE_AUDIO_RESAMPLER_H_

#include <cstdint>

extern "C" {
#include <libavutil/frame.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}

namespace video_pipeline
{

class AudioResampler
{
public:
  AudioResampler();
  ~AudioResampler();

  AudioResampler(const AudioResampler &) = delete;
  AudioResampler & operator=(const AudioResampler &) = delete;

  // 根据输入、输出格式初始化重采样器；声道布局使用 FFmpeg 的位掩码。
  int open(
    int input_rate, AVSampleFormat input_format, std::int64_t input_layout,
    int output_rate, AVSampleFormat output_format, std::int64_t output_layout);

  // 返回输出的每声道采样数，失败时返回负的 FFmpeg 错误码。
  // input 为 nullptr 时排出缓冲；重复调用直至返回 0。
  // 输出帧的时间戳需由调用者根据采样数和时间基计算。
  int convert(const AVFrame * input, AVFrame * output);

  void close();

private:
  SwrContext * resample_context_;
  int input_rate_;
  AVSampleFormat input_format_;
  std::int64_t input_layout_;
  int output_rate_;
  AVSampleFormat output_format_;
  std::int64_t output_layout_;
};

}  // 结束 video_pipeline 命名空间

#endif
