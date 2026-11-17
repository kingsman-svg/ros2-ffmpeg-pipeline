# 同一次解码队列采样中的包数、字节数和容量上限。
uint64 packet_count
uint64 max_packets
uint64 byte_count
uint64 max_bytes

# 队列内最老数据包已等待的毫秒数，空队列时为 0。
uint64 oldest_wait_ms
