#include "video_pipeline/video_encoder.h"

#include <cerrno>

namespace video_pipeline
{

VideoEncoder::VideoEncoder()
: codec_context_(nullptr)
{
}

VideoEncoder::~VideoEncoder()
{
  close();
}

int VideoEncoder::open(const VideoEncoderConfig & config)
{
  close();
  if (config.width <= 0 || config.height <= 0 || config.frames_per_second <= 0 ||
    config.bit_rate <= 0 || config.gop_size <= 0 || config.max_b_frames < 0 ||
    config.pixel_format == AV_PIX_FMT_NONE)
  {
    return AVERROR(EINVAL);
  }

  const AVCodec * codec = nullptr;
  if (config.encoder_name.empty()) {
    codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  } else {
    codec = avcodec_find_encoder_by_name(config.encoder_name.c_str());
  }
  if (codec == nullptr || codec->id != AV_CODEC_ID_H264 || codec->type != AVMEDIA_TYPE_VIDEO) {
    return AVERROR_ENCODER_NOT_FOUND;
  }

  codec_context_ = avcodec_alloc_context3(codec);
  if (codec_context_ == nullptr) {
    return AVERROR(ENOMEM);
  }

  codec_context_->width = config.width;
  codec_context_->height = config.height;
  codec_context_->time_base = AVRational{1, config.frames_per_second};
  codec_context_->framerate = AVRational{config.frames_per_second, 1};
  codec_context_->bit_rate = config.bit_rate;
  codec_context_->gop_size = config.gop_size;
  codec_context_->max_b_frames = config.max_b_frames;
  codec_context_->pix_fmt = config.pixel_format;
  if (config.global_header) {
    codec_context_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  }

  const int result = avcodec_open2(codec_context_, codec, nullptr);
  if (result < 0) {
    close();
    return result;
  }
  return 0;
}

int VideoEncoder::send_frame(const AVFrame * frame)
{
  if (codec_context_ == nullptr) {
    return AVERROR(EINVAL);
  }
  if (frame != nullptr &&
    (frame->width != codec_context_->width || frame->height != codec_context_->height ||
    frame->format != codec_context_->pix_fmt))
  {
    return AVERROR(EINVAL);
  }
  return avcodec_send_frame(codec_context_, frame);
}

int VideoEncoder::receive_packet(AVPacket * packet)
{
  if (codec_context_ == nullptr || packet == nullptr) {
    return AVERROR(EINVAL);
  }
  return avcodec_receive_packet(codec_context_, packet);
}

int VideoEncoder::copy_codec_parameters(AVCodecParameters * parameters) const
{
  if (codec_context_ == nullptr || parameters == nullptr) {
    return AVERROR(EINVAL);
  }
  return avcodec_parameters_from_context(parameters, codec_context_);
}

AVRational VideoEncoder::time_base() const
{
  if (codec_context_ == nullptr) {
    return AVRational{0, 1};
  }
  return codec_context_->time_base;
}

void VideoEncoder::close()
{
  if (codec_context_ != nullptr) {
    avcodec_free_context(&codec_context_);
  }
}

}  // 结束 video_pipeline 命名空间
