#ifndef VIDEO_PIPELINE_FFMPEG_RESOURCES_H_
#define VIDEO_PIPELINE_FFMPEG_RESOURCES_H_

#include <stdexcept>
#include <string>

extern "C" {
#include <libavcodec/packet.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
}

namespace video_pipeline
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

inline std::runtime_error ffmpeg_error(const std::string & action, int error)
{
  char message[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(error, message, sizeof(message));
  return std::runtime_error(action + "：" + message);
}

}  // 结束 video_pipeline 命名空间

#endif
