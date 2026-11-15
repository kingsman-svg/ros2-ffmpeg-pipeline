# 编码帧对应的原始图像头部。
std_msgs/Header header

# PTS 和 DTS 的单位都是 time_base_num / time_base_den 秒。
int64 pts
int64 dts
int32 time_base_num
int32 time_base_den
bool key_frame

# 为 true 时表示编码结束，此时其余字段无效。
bool end_of_stream

# H.264 Annex B 数据，NAL 单元带起始码。
uint8[] data
