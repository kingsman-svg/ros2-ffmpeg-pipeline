# 同一次解码队列采样中的包数、字节数和容量上限。
uint64 packet_count
uint64 max_packets
uint64 byte_count
uint64 max_bytes

# 队列内最老数据包已等待的毫秒数，空队列时为 0。
uint64 oldest_wait_ms

# 解码节点启动以来的单调时间。用两次采样的差值计算速率。
uint64 sample_elapsed_us

# 从节点启动起累计成功入队、成功取出的数据包数，以及已发布图像数。
uint64 total_enqueued_packets
uint64 total_dequeued_packets
uint64 total_published_frames

# 累计耗时，单位为微秒。解码阶段包含送包和取帧，发布阶段包含构建 ROS 图像消息。
uint64 total_decode_time_us
uint64 total_convert_time_us
uint64 total_publish_time_us
