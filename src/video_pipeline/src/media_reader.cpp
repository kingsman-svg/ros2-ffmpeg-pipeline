#include "video_pipeline/media_reader.h"

#include <cerrno>

namespace video_pipeline
{

MediaReader::MediaReader()
: format_context_(nullptr), stream_index_(-1)
{
}

MediaReader::~MediaReader()
{
  close();
}

int MediaReader::open(const std::string & path, AVMediaType media_type)
{
  close();
  if (media_type != AVMEDIA_TYPE_VIDEO && media_type != AVMEDIA_TYPE_AUDIO) {
    return AVERROR(EINVAL);
  }

  int result = avformat_open_input(&format_context_, path.c_str(), nullptr, nullptr);
  if (result < 0) {
    return result;
  }

  result = avformat_find_stream_info(format_context_, nullptr);
  if (result < 0) {
    close();
    return result;
  }

  result = av_find_best_stream(format_context_, media_type, -1, -1, nullptr, 0);
  if (result < 0) {
    close();
    return result;
  }

  stream_index_ = result;
  return 0;
}

int MediaReader::read_packet(AVPacket * packet)
{
  if (format_context_ == nullptr || packet == nullptr) {
    return AVERROR(EINVAL);
  }

  av_packet_unref(packet);
  while (true) {
    const int result = av_read_frame(format_context_, packet);
    if (result < 0) {
      return result;
    }
    if (packet->stream_index == stream_index_) {
      return 0;
    }
    av_packet_unref(packet);
  }
}

const AVCodecParameters * MediaReader::codec_parameters() const
{
  if (format_context_ == nullptr || stream_index_ < 0) {
    return nullptr;
  }
  return format_context_->streams[stream_index_]->codecpar;
}

AVRational MediaReader::time_base() const
{
  if (format_context_ == nullptr || stream_index_ < 0) {
    return AVRational{0, 1};
  }
  return format_context_->streams[stream_index_]->time_base;
}

void MediaReader::close()
{
  if (format_context_ != nullptr) {
    avformat_close_input(&format_context_);
  }
  stream_index_ = -1;
}

}  // 结束 video_pipeline 命名空间
