#include "video_pipeline/audio_resampler.h"

#include <cerrno>

extern "C" {
#include <libavutil/channel_layout.h>
#include <libavutil/error.h>
}

namespace video_pipeline
{

AudioResampler::AudioResampler()
: resample_context_(nullptr), input_rate_(0), input_format_(AV_SAMPLE_FMT_NONE),
  input_layout_(0), output_rate_(0), output_format_(AV_SAMPLE_FMT_NONE), output_layout_(0)
{
}

AudioResampler::~AudioResampler()
{
  close();
}

int AudioResampler::open(
  int input_rate, AVSampleFormat input_format, std::int64_t input_layout,
  int output_rate, AVSampleFormat output_format, std::int64_t output_layout)
{
  close();
  if (input_rate <= 0 || output_rate <= 0 || input_layout <= 0 || output_layout <= 0 ||
    av_get_bytes_per_sample(input_format) <= 0 || av_get_bytes_per_sample(output_format) <= 0)
  {
    return AVERROR(EINVAL);
  }

  resample_context_ = swr_alloc_set_opts(
    nullptr, output_layout, output_format, output_rate,
    input_layout, input_format, input_rate, 0, nullptr);
  if (resample_context_ == nullptr) {
    return AVERROR(ENOMEM);
  }

  const int result = swr_init(resample_context_);
  if (result < 0) {
    close();
    return result;
  }

  input_rate_ = input_rate;
  input_format_ = input_format;
  input_layout_ = input_layout;
  output_rate_ = output_rate;
  output_format_ = output_format;
  output_layout_ = output_layout;
  return 0;
}

int AudioResampler::convert(const AVFrame * input, AVFrame * output)
{
  if (resample_context_ == nullptr || output == nullptr || input == output) {
    return AVERROR(EINVAL);
  }
  if (input != nullptr &&
    (input->nb_samples <= 0 || input->sample_rate != input_rate_ ||
    input->format != input_format_ || input->extended_data == nullptr ||
    (input->channel_layout != 0 && input->channel_layout != static_cast<std::uint64_t>(input_layout_)) ||
    input->channels != av_get_channel_layout_nb_channels(input_layout_)))
  {
    return AVERROR(EINVAL);
  }

  const int input_samples = input == nullptr ? 0 : input->nb_samples;
  const int capacity = swr_get_out_samples(resample_context_, input_samples);
  if (capacity < 0) {
    return capacity;
  }

  av_frame_unref(output);
  output->sample_rate = output_rate_;
  output->format = output_format_;
  output->channel_layout = static_cast<std::uint64_t>(output_layout_);
  output->channels = av_get_channel_layout_nb_channels(output_layout_);
  if (capacity == 0) {
    return 0;
  }

  output->nb_samples = capacity;
  int result = av_frame_get_buffer(output, 0);
  if (result < 0) {
    return result;
  }

  const std::uint8_t ** input_data = nullptr;
  if (input != nullptr) {
    input_data = const_cast<const std::uint8_t **>(input->extended_data);
  }
  result = swr_convert(resample_context_, output->extended_data, capacity, input_data, input_samples);
  if (result < 0) {
    av_frame_unref(output);
    return result;
  }
  output->nb_samples = result;
  return result;
}

void AudioResampler::close()
{
  swr_free(&resample_context_);
  input_rate_ = 0;
  input_format_ = AV_SAMPLE_FMT_NONE;
  input_layout_ = 0;
  output_rate_ = 0;
  output_format_ = AV_SAMPLE_FMT_NONE;
  output_layout_ = 0;
}

}  // 结束 video_pipeline 命名空间
