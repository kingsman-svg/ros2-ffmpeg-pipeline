#ifndef VIDEO_PIPELINE__H264_ANNEX_B_PARSER_H_
#define VIDEO_PIPELINE__H264_ANNEX_B_PARSER_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace video_pipeline
{

class H264AnnexBParser
{
public:
  struct StartPacketPlan
  {
    bool ready = false;
    std::size_t insertion_offset = 0;
    const std::vector<std::uint8_t> * sps = nullptr;
    const std::vector<std::uint8_t> * pps = nullptr;
  };

  // 输入是包含完整 NAL 单元的 Annex B 包，返回的缓存引用有效至下次解析。
  StartPacketPlan inspect_start_packet(const std::vector<std::uint8_t> & data);

private:
  struct PictureParameterSet
  {
    std::uint32_t sps_id = 0;
    std::vector<std::uint8_t> nal;
  };

  std::array<std::vector<std::uint8_t>, 32> sequence_parameter_sets_;
  std::array<PictureParameterSet, 256> picture_parameter_sets_;
};

}  // 结束 video_pipeline 命名空间

#endif  // VIDEO_PIPELINE__H264_ANNEX_B_PARSER_H_
