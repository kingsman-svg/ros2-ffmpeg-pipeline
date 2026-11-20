#include "video_pipeline/h264_annex_b_parser.h"

namespace
{

constexpr std::uint8_t idr_slice = 5;
constexpr std::uint8_t sps_nal = 7;
constexpr std::uint8_t pps_nal = 8;
constexpr std::uint8_t access_unit_delimiter = 9;

std::size_t find_start_code(
  const std::vector<std::uint8_t> & data, std::size_t from, std::size_t & code_size)
{
  for (std::size_t offset = from; offset + 2 < data.size(); ++offset) {
    if (data[offset] != 0 || data[offset + 1] != 0) {
      continue;
    }
    if (data[offset + 2] == 1) {
      code_size = 3;
      return offset;
    }
    if (offset + 3 < data.size() && data[offset + 2] == 0 && data[offset + 3] == 1) {
      code_size = 4;
      return offset;
    }
  }
  return data.size();
}

class RbspBitReader
{
public:
  RbspBitReader(const std::vector<std::uint8_t> & data, std::size_t first, std::size_t end)
  : data_(data), offset_(first), end_(end) {}

  bool read_bits(unsigned int count, std::uint32_t & value)
  {
    value = 0;
    for (unsigned int index = 0; index < count; ++index) {
      if (bits_left_ == 0) {
        if (offset_ == end_) {
          return false;
        }
        // 跳过防竞争字节，缓存中的原始 NAL 字节仍保留它。
        if (zero_bytes_ == 2 && data_[offset_] == 3) {
          ++offset_;
          if (offset_ == end_ || data_[offset_] > 3) {
            return false;
          }
          zero_bytes_ = 0;
        }
        current_byte_ = data_[offset_++];
        if (current_byte_ == 0) {
          ++zero_bytes_;
        } else {
          zero_bytes_ = 0;
        }
        bits_left_ = 8;
      }
      --bits_left_;
      value = (value << 1) | ((current_byte_ >> bits_left_) & 1U);
    }
    return true;
  }

  bool read_unsigned_exp_golomb(std::uint32_t & value)
  {
    unsigned int leading_zeros = 0;
    std::uint32_t bit = 0;
    while (read_bits(1, bit) && bit == 0) {
      ++leading_zeros;
      if (leading_zeros == 32) {
        return false;
      }
    }
    std::uint32_t suffix = 0;
    if (bit == 0 || !read_bits(leading_zeros, suffix)) {
      return false;
    }
    value = ((1U << leading_zeros) - 1U) + suffix;
    return true;
  }

private:
  const std::vector<std::uint8_t> & data_;
  std::size_t offset_;
  const std::size_t end_;
  unsigned int bits_left_ = 0;
  unsigned int zero_bytes_ = 0;
  std::uint8_t current_byte_ = 0;
};

}  // 结束匿名命名空间

namespace video_pipeline
{

H264AnnexBParser::StartPacketPlan H264AnnexBParser::inspect_start_packet(
  const std::vector<std::uint8_t> & data)
{
  // 记录参数在本包中的位置，判断原包内是否已经先有 SPS、再有 PPS。
  std::array<std::size_t, 32> sps_positions{};
  std::array<std::size_t, 256> pps_positions{};
  std::size_t start_code_size = 0;
  const std::size_t first_start = find_start_code(data, 0, start_code_size);
  std::size_t insertion_offset = 0;
  bool saw_picture = false;

  std::size_t end = first_start;
  for (std::size_t start = first_start; start < data.size(); start = end) {
    const std::size_t header = start + start_code_size;
    end = find_start_code(data, header, start_code_size);
    if (header >= end || (data[header] & 0x80) != 0) {
      continue;
    }

    const std::uint8_t type = data[header] & 0x1f;
    RbspBitReader reader(data, header + 1, end);
    std::uint32_t sps_id = 0;
    std::uint32_t pps_id = 0;
    std::uint32_t unused_field = 0;
    if (type == sps_nal) {
      // SPS 的前三个字节是档次、约束标志和级别，接下来才是 SPS ID。
      if (reader.read_bits(24, unused_field) && reader.read_unsigned_exp_golomb(sps_id) &&
        sps_id < sequence_parameter_sets_.size())
      {
        sequence_parameter_sets_[sps_id].assign(data.data() + start, data.data() + end);
        sps_positions[sps_id] = start + 1;
      }
      continue;
    }
    if (type == pps_nal) {
      if (reader.read_unsigned_exp_golomb(pps_id) && reader.read_unsigned_exp_golomb(sps_id) &&
        pps_id < picture_parameter_sets_.size() && sps_id < sequence_parameter_sets_.size())
      {
        auto & pps = picture_parameter_sets_[pps_id];
        pps.sps_id = sps_id;
        pps.nal.assign(data.data() + start, data.data() + end);
        pps_positions[pps_id] = start + 1;
      }
      continue;
    }
    if (type == access_unit_delimiter && start == first_start) {
      // 如果有访问单元分隔符，就让它继续位于整个访问单元的开头。
      insertion_offset = end;
    }
    if (type < 1 || type > idr_slice || saw_picture) {
      continue;
    }
    saw_picture = true;
    // 一个包对应一帧，首个 slice 决定能否从这个包开始解码。
    // slice header 依次存放首个宏块位置、slice 类型和 PPS ID。
    if (type != idr_slice || !reader.read_unsigned_exp_golomb(unused_field) ||
      !reader.read_unsigned_exp_golomb(unused_field) || !reader.read_unsigned_exp_golomb(pps_id) ||
      pps_id >= picture_parameter_sets_.size())
    {
      continue;
    }
    const auto & pps = picture_parameter_sets_[pps_id];
    const auto & sps = sequence_parameter_sets_[pps.sps_id];
    if (pps.nal.empty() || sps.empty()) {
      continue;
    }

    StartPacketPlan plan;
    plan.ready = true;
    if (sps_positions[pps.sps_id] == 0 || sps_positions[pps.sps_id] >= pps_positions[pps_id]) {
      plan.insertion_offset = insertion_offset;
      plan.sps = &sps;
      plan.pps = &pps.nal;
    }
    return plan;
  }
  return StartPacketPlan{};
}

}  // 结束 video_pipeline 命名空间
