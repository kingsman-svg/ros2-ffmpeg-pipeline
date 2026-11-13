#include "video_pipeline/media_decoder.h"

#include <cerrno>

namespace video_pipeline
{

MediaDecoder::MediaDecoder()
: codec_context_(nullptr)
{
}

MediaDecoder::~MediaDecoder()
{
  close();
}

int MediaDecoder::open(const AVCodecParameters * parameters)
{
  close();
  if (parameters == nullptr ||
    (parameters->codec_type != AVMEDIA_TYPE_VIDEO && parameters->codec_type != AVMEDIA_TYPE_AUDIO))
  {
    return AVERROR(EINVAL);
  }

  const AVCodec * codec = avcodec_find_decoder(parameters->codec_id);
  if (codec == nullptr) {
    return AVERROR_DECODER_NOT_FOUND;
  }

  codec_context_ = avcodec_alloc_context3(codec);
  if (codec_context_ == nullptr) {
    return AVERROR(ENOMEM);
  }

  int result = avcodec_parameters_to_context(codec_context_, parameters);
  if (result < 0) {
    close();
    return result;
  }

  result = avcodec_open2(codec_context_, codec, nullptr);
  if (result < 0) {
    close();
    return result;
  }
  return 0;
}

int MediaDecoder::send_packet(const AVPacket * packet)
{
  if (codec_context_ == nullptr) {
    return AVERROR(EINVAL);
  }
  return avcodec_send_packet(codec_context_, packet);
}

int MediaDecoder::receive_frame(AVFrame * frame)
{
  if (codec_context_ == nullptr || frame == nullptr) {
    return AVERROR(EINVAL);
  }
  return avcodec_receive_frame(codec_context_, frame);
}

void MediaDecoder::close()
{
  if (codec_context_ != nullptr) {
    avcodec_free_context(&codec_context_);
  }
}

}  // 结束 video_pipeline 命名空间
