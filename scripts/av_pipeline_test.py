"""验证 PCM 采样连续、Opus 首尾裁剪及音视频同步输出。"""

from pathlib import Path
from math import sqrt
import os
from struct import iter_unpack
import subprocess
import tempfile
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import UInt64
from video_pipeline_msgs.msg import OpusPacket, PcmAudio

from queue_pressure_test import run_launch


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def observe(command, topics, domain, log):
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain,
               signal_handler_options=SignalHandlerOptions.NO)
    node = Node("av_test_observer", context=context)
    received = {topic: [] for topic in topics}
    for topic, message_type in topics.items():
        node.create_subscription(
            message_type, topic,
            lambda message, topic=topic: received[topic].append((time.monotonic_ns(), message)),
            500,
        )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), RCUTILS_COLORIZED_OUTPUT="0")
    try:
        code = run_launch(command, env, log, executor)
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
    if code != 0:
        raise RuntimeError(f"运行失败，退出码 {code}，日志在 {log}")
    return received


def check_pcm(observed, expected_samples, expected_start_ns):
    messages = [message for _, message in observed]
    assert messages and messages[-1].end_of_stream, "缺少 PCM 结束消息"
    sample = 0
    for index, message in enumerate(messages):
        assert message.first_sample_index == sample
        expected_stamp = expected_start_ns + round(sample * 1_000_000_000 / 48000)
        assert abs(stamp_ns(message) - expected_stamp) <= 1
        assert (message.sample_rate, message.channels, message.sample_format) == (48000, 2, "s16le")
        assert len(message.data) == 4 * message.samples_per_channel
        assert message.end_of_stream == (index == len(messages) - 1)
        sample += message.samples_per_channel
    assert sample == expected_samples, (sample, expected_samples)
    return messages


def first_channel(observed):
    data = b"".join(message.data for _, message in observed)
    return [left for left, _ in iter_unpack("<hh", data)]


def main():
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    destination = Path(tempfile.mkdtemp(prefix="opus-av-integration-"))
    print(f"音视频测试日志 {destination}", flush=True)
    wav = destination / "audio.wav"
    subprocess.run([
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i",
        "sine=frequency=440:sample_rate=44100:duration=1.01", "-c:a", "pcm_s16le", str(wav),
    ], check=True)
    raw = observe(
        ["ros2", "run", "video_pipeline", "audio_file_source_node", "--ros-args",
         "-p", f"input_path:={wav}"],
        {"/audio/pcm_raw": PcmAudio}, 173, destination / "pcm.log",
    )
    pcm = check_pcm(raw["/audio/pcm_raw"], 48480, 0)
    assert pcm[-2].samples_per_channel == 480
    print("PCM 重采样通过，每声道 48480 个采样，尾块 480", flush=True)

    av = destination / "offset.mkv"
    subprocess.run([
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i",
        "testsrc2=size=160x120:rate=2:duration=2", "-itsoffset", "0.350", "-f", "lavfi", "-i",
        "sine=frequency=440:sample_rate=44100:duration=1.31", "-map", "0:v", "-map", "1:a",
        "-c:v", "mpeg4", "-c:a", "pcm_s16le", str(av),
    ], check=True)
    received = observe(
        ["ros2", "launch", "video_pipeline", "video_pipeline.launch.py", f"input_path:={av}",
         "fps:=2", "with_audio:=true"],
        {"/audio/pcm_raw": PcmAudio, "/audio/opus": OpusPacket, "/audio/decoded_pcm": PcmAudio,
         "/av/audio": PcmAudio, "/av/video": Image, "/av/video_end": UInt64},
        174, destination / "av.log",
    )
    for topic in ("/audio/pcm_raw", "/audio/decoded_pcm", "/av/audio"):
        check_pcm(received[topic], 62880, 350_000_000)
    opus = [message for _, message in received["/audio/opus"]]
    assert opus[0].skip_samples > 0
    assert opus[-1].end_of_stream
    assert sum(message.samples_per_channel for message in opus) == 62880
    # 跳过首尾瞬态，核对编解码后的波形仍在正确的采样位置。
    original = first_channel(received["/audio/pcm_raw"])[4800:-4800]
    decoded = first_channel(received["/audio/decoded_pcm"])[4800:-4800]
    energy = sqrt(sum(value * value for value in original) * sum(value * value for value in decoded))
    assert energy > 0 and sum(a * b for a, b in zip(original, decoded)) / energy > 0.95
    assert [message.data for _, message in received["/av/video_end"]] == [4]
    frames = received["/av/video"]
    assert [stamp_ns(message) for _, message in frames] == [
        index * 500_000_000 for index in range(4)
    ]
    offset_ns = received["/av/audio"][0][0] - frames[0][0]
    assert abs(offset_ns - 350_000_000) < 70_000_000, offset_ns
    observed = [(instant - frames[0][0]) - stamp_ns(message)
                for instant, message in frames + received["/av/audio"][:-1]]
    assert max(map(abs, observed)) < 70_000_000, observed
    print(f"Opus 解码并同步 62880 个采样/声道和 4 帧，实测音轨偏移 {offset_ns / 1e6:.1f} ms")


if __name__ == "__main__":
    main()
