#define _GNU_SOURCE

#include <dlfcn.h>
#include <errno.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "rcl/publisher.h"

typedef rcl_ret_t (*PublishFunction)(
  const rcl_publisher_t *, const void *, rmw_publisher_allocation_t *);

rcl_ret_t rcl_publish(
  const rcl_publisher_t * publisher,
  const void * message,
  rmw_publisher_allocation_t * allocation)
{
  PublishFunction publish = (PublishFunction)dlsym(RTLD_NEXT, "rcl_publish");
  if (publish == NULL) {
    return RCL_RET_ERROR;
  }

  // 只干预测试指定话题的发布。
  const char * target = getenv("QUEUE_PRESSURE_TOPIC");
  const char * topic = rcl_publisher_get_topic_name(publisher);
  if (target == NULL || topic == NULL || strcmp(topic, target) != 0) {
    return publish(publisher, message, allocation);
  }

  const char * start = getenv("QUEUE_PRESSURE_START_AFTER");
  const char * count = getenv("QUEUE_PRESSURE_COUNT");
  if (start == NULL || count == NULL) {
    return publish(publisher, message, allocation);
  }

  static atomic_ulong publish_count = 0;
  const unsigned long index = atomic_fetch_add_explicit(&publish_count, 1, memory_order_relaxed);
  const unsigned long start_after = strtoul(start, NULL, 10);
  if (index < start_after || index - start_after >= strtoul(count, NULL, 10)) {
    return publish(publisher, message, allocation);
  }
  const char * drop = getenv("QUEUE_PRESSURE_DROP");
  if (drop != NULL && strcmp(drop, "1") == 0) {
    // 模拟反馈短时丢失，让发布线程继续运行。
    return RCL_RET_OK;
  }

  const char * delay = getenv("QUEUE_PRESSURE_DELAY_MS");
  if (delay != NULL) {
    const unsigned long delay_ms = strtoul(delay, NULL, 10);
    struct timespec remaining = {delay_ms / 1000, (delay_ms % 1000) * 1000000};
    while (nanosleep(&remaining, &remaining) == -1 && errno == EINTR) {
      // 被信号中断时补足剩余的等待时间。
    }
  }

  return publish(publisher, message, allocation);
}
