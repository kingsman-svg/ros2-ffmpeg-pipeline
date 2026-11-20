#include "video_pipeline/h264_annex_b_parser.h"

#include <initializer_list>
#include <stdexcept>

namespace
{

using Bytes = std::vector<std::uint8_t>;

void check(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

Bytes make_packet(std::initializer_list<Bytes> nals)
{
  Bytes packet;
  for (const auto & nal : nals) {
    packet.insert(packet.end(), nal.begin(), nal.end());
  }
  return packet;
}

}  // 结束匿名命名空间

int main()
{
  // 这些 NAL 只包含供解析器读取的头部字段，实际解码使用完整码流另行验证。
  const Bytes sps0{0, 0, 0, 1, 0x67, 0x42, 0, 0x1e, 0xc0};
  const Bytes sps1{0, 0, 0, 1, 0x67, 0x42, 0, 0x1e, 0x50};
  const Bytes pps0{0, 0, 1, 0x68, 0xe0};
  const Bytes pps3{0, 0, 0, 1, 0x68, 0x22, 0x80};
  const Bytes idr3{0, 0, 1, 0x65, 0xb2, 0x40};
  const Bytes predicted3{0, 0, 1, 0x41, 0xc9};
  const Bytes aud{0, 0, 0, 1, 0x09, 0xf0};
  const Bytes truncated_idr{0, 0, 1, 0x65};
  video_pipeline::H264AnnexBParser parser;

  check(!parser.inspect_start_packet(idr3).ready, "没有参数集时应继续等待");
  check(!parser.inspect_start_packet(make_packet({sps0, pps0, pps3})).ready,
    "只有参数集的包应写入缓存并继续等待");
  check(!parser.inspect_start_packet(idr3).ready, "PPS 对应的 SPS 缺失时应继续等待");
  check(!parser.inspect_start_packet(make_packet({sps1, predicted3})).ready,
    "参数齐全的预测帧也不是解码起点");
  check(!parser.inspect_start_packet(truncated_idr).ready, "截断的 IDR 应继续等待");

  const auto from_cache = parser.inspect_start_packet(make_packet({aud, idr3}));
  check(from_cache.ready && from_cache.insertion_offset == aud.size() &&
    from_cache.sps != nullptr && *from_cache.sps == sps1 &&
    from_cache.pps != nullptr && *from_cache.pps == pps3,
    "应在分隔符后补入 IDR 实际引用的 SPS 和 PPS");

  const auto inline_parameters = parser.inspect_start_packet(make_packet({sps1, pps3, idr3}));
  check(inline_parameters.ready && inline_parameters.sps == nullptr &&
    inline_parameters.pps == nullptr, "参数已经齐全的 IDR 应直接使用原包");

  video_pipeline::H264AnnexBParser late_parameters;
  check(!late_parameters.inspect_start_packet(make_packet({idr3, sps1, pps3})).ready,
    "出现在首个 slice 后面的参数应留到下一个 IDR 使用");
  check(late_parameters.inspect_start_packet(idr3).ready, "下一个 IDR 可以使用缓存参数");
}
