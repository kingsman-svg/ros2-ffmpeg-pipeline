#include "video_pipeline/bitrate_controller.h"

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace
{

void check(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // 结束匿名命名空间

int main()
{
  using video_pipeline::BitrateChange;
  video_pipeline::BitrateController controller(2000000);
  video_pipeline::BitrateMetrics metrics;
  metrics.feedback_valid = true;
  metrics.encoded_packets_per_second = 30;
  metrics.published_packets_per_second = 30;
  const auto tick = [&]() {
      metrics.sampled_at += std::chrono::milliseconds(200);
      return controller.update(metrics);
    };

  check(controller.bit_rate() == 1000000, "初始码率应为上限的一半");
  for (int i = 0; i < 5; ++i) {
    check(tick().change == BitrateChange::Hold, "健康窗口不足一秒时应保持目标");
  }
  const auto slow_start = tick();
  check(slow_start.change == BitrateChange::SlowStart && slow_start.bit_rate == 1500000,
    "慢启动应增长到初始阈值");
  for (int i = 0; i < 4; ++i) {
    check(tick().change == BitrateChange::Hold, "达到阈值后应按窗口增速");
  }
  const auto increase = tick();
  check(increase.change == BitrateChange::Increase && increase.bit_rate == 1600000,
    "达到阈值后应按固定步长增长");

  metrics.send_wait_ms = 400;
  metrics.published_packets_per_second = 24;
  metrics.encoded_bits_per_second = 1600000;
  metrics.published_bits_per_second = 1000000;
  for (int i = 0; i < 5; ++i) {
    check(tick().change == BitrateChange::Hold, "短时积压不应触发降速");
  }
  const auto decrease = tick();
  check(decrease.change == BitrateChange::SendCongestion &&
    decrease.bit_rate == 800000 && decrease.slow_start_threshold == 800000,
    "发送端持续积压时应减半目标码率和阈值");
  for (int i = 0; i < 15; ++i) {
    check(tick().change == BitrateChange::Hold, "同一轮积压应只降速一次");
  }

  metrics.send_wait_ms = 0;
  metrics.published_packets_per_second = 30;
  metrics.feedback_valid = false;
  for (int i = 0; i < 15; ++i) {
    check(tick().change == BitrateChange::Hold, "反馈过期时应保持目标");
  }
  metrics.feedback_valid = true;
  metrics.decode_wait_ms = 350;
  for (int i = 0; i < 15; ++i) {
    check(tick().change == BitrateChange::Hold, "下游积压时应保持目标");
  }
  metrics.decode_wait_ms = 0;
  for (int i = 0; i < 10; ++i) {
    check(tick().change == BitrateChange::Hold, "恢复窗口不足两秒时应保持目标");
  }
  const auto recovered = tick();
  check(recovered.change == BitrateChange::Increase && recovered.bit_rate == 900000,
    "恢复后应从减半后的阈值按固定步长增长");
  std::cout << "码率控制状态转换通过\n";
}
