# 时间戳和序号记录本包解码后首个有效采样的位置。
# 本链路采用 48 kHz 双声道、20 ms 的原始 Opus 包。
std_msgs/Header header
uint64 first_sample_index
uint32 skip_samples
uint32 samples_per_channel
uint8[] data

# 结束消息没有数据，first_sample_index 此时是每声道总采样数。
bool end_of_stream
