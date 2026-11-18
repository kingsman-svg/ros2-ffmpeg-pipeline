#ifndef VIDEO_PIPELINE_MEDIA_READER_H_
#define VIDEO_PIPELINE_MEDIA_READER_H_

#include <cstdint>
#include <string>

extern "C" {
#include <libavformat/avformat.h>
}

namespace video_pipeline
{

class MediaReader
{
public:
  MediaReader();
  ~MediaReader();

  MediaReader(const MediaReader &) = delete;
  MediaReader & operator=(const MediaReader &) = delete;

  // 打开指定类型的音频或视频流；重复打开前会关闭当前输入。
  int open(const std::string & path, AVMediaType media_type);

  // packet 由调用者分配和释放；读到文件末尾时返回 AVERROR_EOF。
  int read_packet(AVPacket * packet);

  // 返回的指针由 MediaReader 持有，关闭或重新打开输入后失效。
  const AVCodecParameters * codec_parameters() const;
  AVRational time_base() const;
  // 将流时间戳换算到文件的共同时间轴。缺省时使用流起点。
  std::int64_t media_time_ns(std::int64_t pts) const;

  void close();

private:
  AVFormatContext * format_context_;
  int stream_index_;
};

}  // 结束 video_pipeline 命名空间

#endif
