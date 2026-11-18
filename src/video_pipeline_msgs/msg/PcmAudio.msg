# 时间戳记录本块首个有效采样，使用源节点的媒体时间。
std_msgs/Header header
uint32 sample_rate
uint8 channels
string sample_format
uint64 first_sample_index
uint32 samples_per_channel
uint8[] data

# 结束消息没有数据，first_sample_index 此时是每声道总采样数。
bool end_of_stream
