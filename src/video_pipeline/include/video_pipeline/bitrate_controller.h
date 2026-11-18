#ifndef VIDEO_PIPELINE_BITRATE_CONTROLLER_H_
#define VIDEO_PIPELINE_BITRATE_CONTROLLER_H_

#include <chrono>
#include <cstdint>

namespace video_pipeline
{

struct BitrateMetrics
{
  std::chrono::steady_clock::time_point sampled_at;
  std::uint64_t send_wait_ms = 0;
  double encoded_packets_per_second = 0;
  double published_packets_per_second = 0;
  double encoded_bits_per_second = 0;
  double published_bits_per_second = 0;
  bool feedback_valid = false;
  std::uint64_t decode_wait_ms = 0;
  double decode_enqueue_packets_per_second = 0;
  double decode_dequeue_packets_per_second = 0;
  double decode_busy_fraction = 0;
};

enum class BitrateChange {Hold, SlowStart, Increase, SendCongestion};

struct BitrateDecision
{
  std::int64_t bit_rate;
  std::int64_t slow_start_threshold;
  BitrateChange change;
};

class BitrateController
{
public:
  explicit BitrateController(std::int64_t maximum_bit_rate);
  std::int64_t bit_rate() const {return bit_rate_;}
  BitrateDecision update(const BitrateMetrics & metrics);

private:
  enum class QueueState {Uncertain, Healthy, SendCongested};
  static QueueState classify(const BitrateMetrics & metrics);

  const std::int64_t maximum_bit_rate_;
  const std::int64_t minimum_bit_rate_;
  const std::int64_t increase_step_;
  std::int64_t bit_rate_;
  std::int64_t slow_start_threshold_;
  bool recovering_ = false;
  QueueState state_ = QueueState::Uncertain;
  std::chrono::steady_clock::time_point state_since_;
  std::chrono::steady_clock::time_point last_sample_;
};

}  // 结束 video_pipeline 命名空间

#endif
