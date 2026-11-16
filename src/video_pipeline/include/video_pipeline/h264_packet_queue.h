#ifndef VIDEO_PIPELINE_H264_PACKET_QUEUE_H_
#define VIDEO_PIPELINE_H264_PACKET_QUEUE_H_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <ostream>
#include <string>

#include "video_pipeline_msgs/msg/h264_packet.hpp"

namespace video_pipeline
{

struct H264PacketQueueStats
{
  std::size_t packet_count;
  std::size_t byte_count;
  std::size_t max_packets;
  std::size_t max_bytes;
  double byte_usage_percent;
  std::chrono::milliseconds oldest_wait;
};

std::ostream & operator<<(std::ostream & output, const H264PacketQueueStats & stats);

class H264PacketQueue
{
public:
  using Packet = video_pipeline_msgs::msg::H264Packet::ConstSharedPtr;

  enum class PopResult {Packet, Finished, Cancelled};

  H264PacketQueue(std::string name, std::int64_t max_packets, std::int64_t max_bytes);

  // 满队列时报告错误，避免丢失后续帧依赖的 H.264 数据包。
  void push(Packet packet);
  PopResult wait_pop(Packet & packet);
  H264PacketQueueStats stats() const;

  // 正常关闭后仍能取出已入队的数据。取消会清空待处理的数据并唤醒消费者。
  void close();
  void cancel();

private:
  struct Entry
  {
    Packet packet;
    std::chrono::steady_clock::time_point enqueued_at;
  };

  enum class State {Open, Closed, Cancelled};

  const std::string name_;
  const std::size_t max_packets_;
  const std::size_t max_bytes_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<Entry> entries_;
  std::size_t byte_count_ = 0;
  State state_ = State::Open;
};

}  // 结束 video_pipeline 命名空间

#endif
