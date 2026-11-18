#include "video_pipeline/bitrate_controller.h"

#include <algorithm>
#include <stdexcept>

namespace video_pipeline
{
namespace
{

constexpr std::int64_t kRateUnit = 1000;
constexpr std::int64_t kMaximumX264BitRate = 2000000000;
constexpr std::uint64_t kHealthyWaitMs = 100;
constexpr std::uint64_t kCongestedWaitMs = 300;
constexpr std::chrono::milliseconds kMaxSampleGap{500};
constexpr std::chrono::seconds kCongestionDuration{1};
constexpr std::chrono::seconds kIncreaseInterval{1};
constexpr std::chrono::seconds kRecoveryDuration{2};

std::int64_t round_rate_down(std::int64_t rate)
{
  return rate / kRateUnit * kRateUnit;
}

std::int64_t validated_maximum(std::int64_t rate)
{
  if (rate < kRateUnit || rate > kMaximumX264BitRate) {
    throw std::invalid_argument("自适应码率上限必须在 1000 至 2000000000 bit/s 之间");
  }
  return round_rate_down(rate);
}

}  // 结束匿名命名空间

BitrateController::BitrateController(std::int64_t maximum_bit_rate)
: maximum_bit_rate_(validated_maximum(maximum_bit_rate)),
  minimum_bit_rate_(std::max(kRateUnit, round_rate_down(maximum_bit_rate_ / 8))),
  increase_step_(std::max(kRateUnit, round_rate_down(maximum_bit_rate_ / 20))),
  bit_rate_(std::max(minimum_bit_rate_, round_rate_down(maximum_bit_rate_ / 2))),
  slow_start_threshold_(std::max(bit_rate_, round_rate_down(maximum_bit_rate_ * 3 / 4)))
{
}

BitrateController::QueueState BitrateController::classify(const BitrateMetrics & metrics)
{
  // 解码端忙且正在积压时，先保留当前码率，给下游恢复的时间。
  const bool decoder_saturated = metrics.decode_busy_fraction >= 0.9 &&
    metrics.decode_enqueue_packets_per_second > metrics.decode_dequeue_packets_per_second;
  if (!metrics.feedback_valid || metrics.decode_wait_ms >= kHealthyWaitMs || decoder_saturated ||
    metrics.encoded_packets_per_second <= 0)
  {
    return QueueState::Uncertain;
  }
  if (metrics.send_wait_ms >= kCongestedWaitMs &&
    metrics.encoded_packets_per_second > metrics.published_packets_per_second &&
    metrics.encoded_bits_per_second > metrics.published_bits_per_second)
  {
    return QueueState::SendCongested;
  }
  if (metrics.send_wait_ms < kHealthyWaitMs) {
    return QueueState::Healthy;
  }
  return QueueState::Uncertain;
}

BitrateDecision BitrateController::update(const BitrateMetrics & metrics)
{
  const QueueState state = classify(metrics);
  const auto gap = metrics.sampled_at - last_sample_;
  if (state != state_ || gap <= std::chrono::steady_clock::duration::zero() ||
    gap > kMaxSampleGap)
  {
    state_ = state;
    state_since_ = metrics.sampled_at;
  }
  last_sample_ = metrics.sampled_at;
  const auto duration = metrics.sampled_at - state_since_;

  if (state == QueueState::SendCongested && !recovering_ && duration >= kCongestionDuration &&
    bit_rate_ > minimum_bit_rate_)
  {
    // 同一轮积压只做一次试降，等队列恢复后再继续探测。
    slow_start_threshold_ = std::max(minimum_bit_rate_, round_rate_down(bit_rate_ / 2));
    bit_rate_ = slow_start_threshold_;
    recovering_ = true;
    return {bit_rate_, slow_start_threshold_, BitrateChange::SendCongestion};
  }

  const auto increase_interval = recovering_ ? kRecoveryDuration : kIncreaseInterval;
  if (state != QueueState::Healthy || duration < increase_interval) {
    return {bit_rate_, slow_start_threshold_, BitrateChange::Hold};
  }
  recovering_ = false;
  if (bit_rate_ >= maximum_bit_rate_) {
    return {bit_rate_, slow_start_threshold_, BitrateChange::Hold};
  }
  state_since_ = metrics.sampled_at;
  if (bit_rate_ < slow_start_threshold_) {
    const auto increase = std::max(kRateUnit, round_rate_down(bit_rate_ / 2));
    bit_rate_ = std::min(slow_start_threshold_, bit_rate_ + increase);
    return {bit_rate_, slow_start_threshold_, BitrateChange::SlowStart};
  }
  bit_rate_ = std::min(maximum_bit_rate_, bit_rate_ + increase_step_);
  return {bit_rate_, slow_start_threshold_, BitrateChange::Increase};
}

}  // 结束 video_pipeline 命名空间
