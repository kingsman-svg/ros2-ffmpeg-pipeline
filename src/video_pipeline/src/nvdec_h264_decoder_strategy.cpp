#include "video_pipeline/nvdec_h264_decoder_strategy.h"

#include <cerrno>
#include <utility>

extern "C" {
#include <libavutil/hwcontext.h>
#include <libavutil/log.h>
#include <libavutil/pixdesc.h>
}

namespace video_pipeline
{

NvdecH264DecoderStrategy::NvdecH264DecoderStrategy(bool allow_software_fallback)
: allow_software_fallback_(allow_software_fallback)
{
}

void NvdecH264DecoderStrategy::CodecContextDeleter::operator()(AVCodecContext * context) const
{
  avcodec_free_context(&context);
}

int NvdecH264DecoderStrategy::open()
{
  decoded_frame_.reset();
  codec_context_.reset();
  hardware_frame_reported_ = false;

  const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (codec == nullptr) {
    return AVERROR_DECODER_NOT_FOUND;
  }
  for (int index = 0; ; ++index) {
    const AVCodecHWConfig * config = avcodec_get_hw_config(codec, index);
    if (config == nullptr) {
      return AVERROR(ENOSYS);
    }
    if (config->device_type == AV_HWDEVICE_TYPE_CUDA && config->pix_fmt == AV_PIX_FMT_CUDA &&
      (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) != 0)
    {
      break;
    }
  }

  std::unique_ptr<AVCodecContext, CodecContextDeleter> context(avcodec_alloc_context3(codec));
  std::unique_ptr<AVFrame, FrameDeleter> frame(av_frame_alloc());
  if (!context || !frame) {
    return AVERROR(ENOMEM);
  }
  // 解码上下文持有 CUDA 设备的引用，释放解码器时一并释放。
  int result = av_hwdevice_ctx_create(
    &context->hw_device_ctx, AV_HWDEVICE_TYPE_CUDA, nullptr, nullptr, 0);
  if (result < 0) {
    return result;
  }
  context->opaque = this;
  context->get_format = choose_pixel_format;
  result = avcodec_open2(context.get(), codec, nullptr);
  if (result < 0) {
    return result;
  }

  codec_context_ = std::move(context);
  decoded_frame_ = std::move(frame);
  return 0;
}

AVPixelFormat NvdecH264DecoderStrategy::choose_pixel_format(
  AVCodecContext * context, const AVPixelFormat * formats)
{
  const auto * strategy = static_cast<const NvdecH264DecoderStrategy *>(context->opaque);
  AVPixelFormat software_format = AV_PIX_FMT_NONE;
  for (const auto * format = formats; *format != AV_PIX_FMT_NONE; ++format) {
    if (*format == AV_PIX_FMT_CUDA) {
      return AV_PIX_FMT_CUDA;
    }
    const AVPixFmtDescriptor * descriptor = av_pix_fmt_desc_get(*format);
    if (descriptor != nullptr && (descriptor->flags & AV_PIX_FMT_FLAG_HWACCEL) == 0) {
      software_format = *format;
    }
  }

  // 硬件格式初始化失败时 FFmpeg 会重新询问，可以用同一份输入接续软件解码。
  if (strategy->allow_software_fallback_ && software_format != AV_PIX_FMT_NONE) {
    av_log(context, AV_LOG_WARNING, "NVDEC 无法使用 CUDA 像素格式，回退到软件解码\n");
    return software_format;
  }
  av_log(context, AV_LOG_ERROR, "NVDEC 无法使用 CUDA 像素格式\n");
  return AV_PIX_FMT_NONE;
}

int NvdecH264DecoderStrategy::send_packet(const AVPacket * packet)
{
  if (!codec_context_) {
    return AVERROR(EINVAL);
  }
  return avcodec_send_packet(codec_context_.get(), packet);
}

int NvdecH264DecoderStrategy::receive_frame(AVFrame * frame)
{
  if (!codec_context_ || frame == nullptr) {
    return AVERROR(EINVAL);
  }
  av_frame_unref(frame);
  int result = avcodec_receive_frame(codec_context_.get(), decoded_frame_.get());
  if (result < 0) {
    return result;
  }
  if (decoded_frame_->format != AV_PIX_FMT_CUDA) {
    if (!allow_software_fallback_) {
      av_frame_unref(decoded_frame_.get());
      return AVERROR(ENOSYS);
    }
    av_frame_move_ref(frame, decoded_frame_.get());
    return 0;
  }

  result = av_hwframe_transfer_data(frame, decoded_frame_.get(), 0);
  if (result >= 0) {
    // 复制 PTS 等属性，解码节点用 PTS 找回原始 ROS 消息头。
    result = av_frame_copy_props(frame, decoded_frame_.get());
  }
  av_frame_unref(decoded_frame_.get());
  if (result < 0) {
    av_frame_unref(frame);
    return result;
  }
  if (!hardware_frame_reported_) {
    av_log(codec_context_.get(), AV_LOG_INFO, "H.264 解码后端为 NVDEC (CUDA)\n");
    hardware_frame_reported_ = true;
  }
  return 0;
}

}  // 结束 video_pipeline 命名空间
