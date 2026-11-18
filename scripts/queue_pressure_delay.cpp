#include <dlfcn.h>
#include <time.h>

#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <cstring>

#include "rcl/publisher.h"
#include "video_pipeline_msgs/msg/h264_packet.hpp"

using PublishFunction = rcl_ret_t (*)(
  const rcl_publisher_t *, const void *, rmw_publisher_allocation_t *);

static void sleep_for_ns(unsigned long long duration)
{
  timespec remaining{
    static_cast<time_t>(duration / 1000000000),
    static_cast<long>(duration % 1000000000)
  };
  while (nanosleep(&remaining, &remaining) == -1 && errno == EINTR) {
    // 被信号中断时补足剩余的等待时间。
  }
}

extern "C" rcl_ret_t rcl_publish(
  const rcl_publisher_t * publisher,
  const void * message,
  rmw_publisher_allocation_t * allocation)
{
  const auto publish = reinterpret_cast<PublishFunction>(dlsym(RTLD_NEXT, "rcl_publish"));
  if (publish == nullptr) {
    return RCL_RET_ERROR;
  }

  // 只干预测试指定话题的发布。
  const char * target = std::getenv("QUEUE_PRESSURE_TOPIC");
  const char * topic = rcl_publisher_get_topic_name(publisher);
  if (target == nullptr || topic == nullptr || std::strcmp(topic, target) != 0) {
    return publish(publisher, message, allocation);
  }

  const char * start = std::getenv("QUEUE_PRESSURE_START_AFTER");
  if (start == nullptr) {
    return publish(publisher, message, allocation);
  }

  static std::atomic<unsigned long> publish_count{0};
  const auto index = publish_count.fetch_add(1, std::memory_order_relaxed);
  const auto start_after = std::strtoul(start, nullptr, 10);
  if (index < start_after) {
    return publish(publisher, message, allocation);
  }

  const char * bit_rate_text = std::getenv("QUEUE_PRESSURE_BIT_RATE");
  if (bit_rate_text != nullptr && message != nullptr && std::strcmp(topic, "/video/h264") == 0) {
    const auto bit_rate = std::strtoull(bit_rate_text, nullptr, 10);
    const auto & packet = *static_cast<const video_pipeline_msgs::msg::H264Packet *>(message);
    if (bit_rate > 0 && !packet.end_of_stream) {
      // 传输时间随 H.264 数据包大小变化，限速持续到视频结束。
      sleep_for_ns(packet.data.size() * 8000000000ULL / bit_rate);
    }
    return publish(publisher, message, allocation);
  }

  const char * count = std::getenv("QUEUE_PRESSURE_COUNT");
  if (count == nullptr || index - start_after >= std::strtoul(count, nullptr, 10)) {
    return publish(publisher, message, allocation);
  }
  const char * drop = std::getenv("QUEUE_PRESSURE_DROP");
  if (drop != nullptr && std::strcmp(drop, "1") == 0) {
    // 模拟反馈短时丢失，让发布线程继续运行。
    return RCL_RET_OK;
  }

  const char * delay = std::getenv("QUEUE_PRESSURE_DELAY_MS");
  if (delay != nullptr) {
    sleep_for_ns(std::strtoull(delay, nullptr, 10) * 1000000ULL);
  }

  return publish(publisher, message, allocation);
}
