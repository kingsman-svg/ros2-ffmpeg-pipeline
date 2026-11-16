#include "video_pipeline/h264_packet_queue.h"

#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace video_pipeline
{
namespace
{

std::size_t checked_limit(std::int64_t limit, const char * parameter)
{
  if (limit <= 0 || static_cast<std::uintmax_t>(limit) > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument(std::string(parameter) + " 必须是有效的正整数");
  }
  return static_cast<std::size_t>(limit);
}

}  // 结束匿名命名空间

std::ostream & operator<<(std::ostream & output, const H264PacketQueueStats & stats)
{
  return output << stats.packet_count << "/" << stats.max_packets << " 包，"
                << stats.byte_count << "/" << stats.max_bytes << " 字节（"
                << stats.byte_usage_percent << "%），最老包等待 " << stats.oldest_wait.count() << " ms";
}

H264PacketQueue::H264PacketQueue(
  std::string name, std::int64_t max_packets, std::int64_t max_bytes)
: name_(std::move(name)),
  max_packets_(checked_limit(max_packets, "queue_max_packets")),
  max_bytes_(checked_limit(max_bytes, "queue_max_bytes"))
{
}

void H264PacketQueue::push(Packet packet)
{
  if (!packet) {
    throw std::invalid_argument(name_ + "收到空指针");
  }
  const std::size_t packet_size = packet->data.size();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::Open) {
      throw std::runtime_error(name_ + "已关闭，无法接收数据包");
    }
    if (entries_.size() >= max_packets_ || packet_size > max_bytes_ - byte_count_) {
      std::ostringstream message;
      message << name_ << "已满，当前 " << entries_.size() << "/" << max_packets_ << " 包，"
              << byte_count_ << "/" << max_bytes_ << " 字节，新包 " << packet_size << " 字节";
      throw std::runtime_error(message.str());
    }
    entries_.push_back({std::move(packet), std::chrono::steady_clock::now()});
    byte_count_ += packet_size;
  }
  ready_.notify_one();
}

H264PacketQueue::PopResult H264PacketQueue::wait_pop(Packet & packet)
{
  // 处理完上一个包后，先释放它的引用再等待下一个包。
  packet.reset();
  std::unique_lock<std::mutex> lock(mutex_);
  ready_.wait(lock, [this]() {return !entries_.empty() || state_ != State::Open;});
  if (state_ == State::Cancelled) {
    return PopResult::Cancelled;
  }
  if (entries_.empty()) {
    return PopResult::Finished;
  }
  packet = std::move(entries_.front().packet);
  entries_.pop_front();
  byte_count_ -= packet->data.size();
  return PopResult::Packet;
}

H264PacketQueueStats H264PacketQueue::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::chrono::milliseconds oldest_wait{0};
  if (!entries_.empty()) {
    oldest_wait = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - entries_.front().enqueued_at);
  }
  const double byte_usage_percent = 100.0 * static_cast<double>(byte_count_) /
    static_cast<double>(max_bytes_);
  return {entries_.size(), byte_count_, max_packets_, max_bytes_, byte_usage_percent, oldest_wait};
}

void H264PacketQueue::close()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::Open) {
      state_ = State::Closed;
    }
  }
  ready_.notify_all();
}

void H264PacketQueue::cancel()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = State::Cancelled;
    entries_.clear();
    byte_count_ = 0;
  }
  ready_.notify_all();
}

}  // 结束 video_pipeline 命名空间
