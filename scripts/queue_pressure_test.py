"""在 ROS 2 容器里验证 H.264 队列积压、排空和容量错误。"""

import argparse
import csv
from dataclasses import dataclass, fields
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from video_pipeline_msgs.msg import H264Packet


FPS = 10
FRAME_COUNT = 80
START_AFTER = 5
DELAY_COUNT = 18
DELAY_MS = 220
CASE_TIMEOUT_SECONDS = 35

QUEUE_NAMES = {"send": "发送", "decode": "解码"}
QUEUE_KEYS = {name: queue for queue, name in QUEUE_NAMES.items()}
DELAY_TOPICS = {"send": "/video/h264", "decode": "/video/decoded_image"}
CASES = (
    "baseline",
    "slow_send",
    "slow_decode",
    "send_overflow",
    "decode_overflow",
)
QUEUE_SAMPLE = re.compile(
    r"\[INFO\] \[(?P<timestamp>\d+(?:\.\d+)?)\] \[[^\]]+\]: "
    r"H\.264 (?P<queue>发送|解码)队列：(?P<packets>\d+)/(?P<max_packets>\d+) 包，"
    r"(?P<bytes>\d+)/(?P<max_bytes>\d+) 字节（(?P<byte_usage_pct>[\d.eE+-]+)%），"
    r"最老包等待 (?P<oldest_wait_ms>\d+) ms"
)
SAMPLE_FIELDS = (
    "case", "timestamp", "queue", "packets", "max_packets", "packet_usage_pct",
    "bytes", "max_bytes", "byte_usage_pct", "oldest_wait_ms",
)


def image_header(header):
    """用时间戳和 frame_id 对照编码前后的同一帧。"""
    return header.stamp.sec, header.stamp.nanosec, header.frame_id


class PipelineObserver(Node):
    """通过实际消息验证帧数、顺序和结束标记。"""

    def __init__(self, context):
        super().__init__("queue_pressure_observer", context=context)
        self.source_headers = []
        self.decoded_headers = []
        self.packets = []
        self.create_subscription(Image, "/video/image_raw", self.on_source, FRAME_COUNT)
        self.create_subscription(Image, "/video/decoded_image", self.on_decoded, FRAME_COUNT)
        self.create_subscription(H264Packet, "/video/h264", self.on_packet, FRAME_COUNT + 1)

    def on_source(self, message):
        self.source_headers.append(image_header(message.header))

    def on_decoded(self, message):
        self.decoded_headers.append(image_header(message.header))

    def on_packet(self, message):
        if message.end_of_stream:
            self.packets.append(None)
            return
        self.packets.append((message.pts, image_header(message.header)))


def build_delay_library(directory):
    """只给压测启动的进程加载延迟库。"""
    source = Path(__file__).with_name("queue_pressure_delay.c")
    library = directory / "queue_pressure_delay.so"
    include_dir = Path(get_package_prefix("rcl")) / "include"
    command = ["cc", "-std=c11", "-shared", "-fPIC", "-I", str(include_dir)]
    for package_dir in sorted(include_dir.iterdir()):
        if package_dir.is_dir():
            command.extend(["-I", str(package_dir)])
    command.extend([str(source), "-o", str(library), "-ldl"])
    subprocess.run(command, check=True)
    return library


def run_launch(command, env, log_path, executor):
    """运行整条链路，超时或中断时清理整组子进程。"""
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            command, env=env, stdout=log_file, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.monotonic() + CASE_TIMEOUT_SECONDS
        try:
            while process.poll() is None:
                if time.monotonic() >= deadline:
                    return None
                executor.spin_once(timeout_sec=0.05)
            # 进程退出时，观察节点还可能有已收到但未回调的消息。
            drain_deadline = time.monotonic() + 0.5
            while time.monotonic() < drain_deadline:
                executor.spin_once(timeout_sec=0.05)
            return process.returncode
        finally:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                except ProcessLookupError:
                    process.wait()


def read_samples(case, log):
    samples = []
    for match in QUEUE_SAMPLE.finditer(log):
        sample = match.groupdict()
        sample["case"] = case
        sample["queue"] = QUEUE_KEYS[sample["queue"]]
        sample["packets"] = int(sample["packets"])
        sample["bytes"] = int(sample["bytes"])
        sample["byte_usage_pct"] = float(sample["byte_usage_pct"])
        sample["oldest_wait_ms"] = int(sample["oldest_wait_ms"])
        sample["packet_usage_pct"] = round(100 * sample["packets"] / int(sample["max_packets"]), 3)
        samples.append(sample)
    return samples


@dataclass
class QueueStats:
    peak_packets: int
    peak_bytes: int
    peak_packet_usage_pct: float
    peak_byte_usage_pct: float
    peak_oldest_wait_ms: int
    drained_after_backlog: bool


def summarize_queue(samples, queue):
    queue_samples = [sample for sample in samples if sample["queue"] == queue]
    if not queue_samples:
        return None
    saw_backlog = False
    drained = False
    for sample in queue_samples:
        if sample["packets"] > 0:
            saw_backlog = True
        elif saw_backlog:
            drained = True
    return QueueStats(
        peak_packets=max(sample["packets"] for sample in queue_samples),
        peak_bytes=max(sample["bytes"] for sample in queue_samples),
        peak_packet_usage_pct=max(sample["packet_usage_pct"] for sample in queue_samples),
        peak_byte_usage_pct=max(sample["byte_usage_pct"] for sample in queue_samples),
        peak_oldest_wait_ms=max(sample["oldest_wait_ms"] for sample in queue_samples),
        drained_after_backlog=drained,
    )


def run_case(case, domain_id, video, delay_library, log_path):
    command = [
        "ros2", "launch", "video_pipeline", "video_pipeline.launch.py",
        f"input_path:={video}", f"fps:={FPS}",
    ]
    env = os.environ.copy()
    env.update(
        ROS_DOMAIN_ID=str(domain_id),
        ROS_LOCALHOST_ONLY="1",
        RCUTILS_COLORIZED_OUTPUT="0",
        RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}",
        PYTHONUNBUFFERED="1",
    )
    slow_queue = None
    overflow_queue = None
    if case.startswith("slow_"):
        slow_queue = case.removeprefix("slow_")
        preload = str(delay_library)
        if env.get("LD_PRELOAD"):
            preload += ":" + env["LD_PRELOAD"]
        env.update(
            LD_PRELOAD=preload,
            QUEUE_PRESSURE_TOPIC=DELAY_TOPICS[slow_queue],
            QUEUE_PRESSURE_START_AFTER=str(START_AFTER),
            QUEUE_PRESSURE_DELAY_COUNT=str(DELAY_COUNT),
            QUEUE_PRESSURE_DELAY_MS=str(DELAY_MS),
        )
    if case.endswith("_overflow"):
        overflow_queue = case.removesuffix("_overflow")
        command.append(f"{overflow_queue}_queue_max_bytes:=1")

    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain_id,
               signal_handler_options=SignalHandlerOptions.NO)
    observer = PipelineObserver(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    try:
        exit_code = run_launch(command, env, log_path, executor)
    finally:
        executor.shutdown()
        observer.destroy_node()
        context.shutdown()

    log = log_path.read_text(encoding="utf-8", errors="replace")
    samples = read_samples(case, log)
    data_packets = [packet for packet in observer.packets if packet is not None]
    eos_order_ok = observer.packets.count(None) == 1 and observer.packets[-1] is None
    expected_packets = list(enumerate(observer.source_headers))
    problems = []
    if exit_code is None:
        problems.append("运行超时")
    elif overflow_queue:
        overflow_text = f"H.264 {QUEUE_NAMES[overflow_queue]}队列已满"
        if exit_code <= 0 or overflow_text not in log:
            problems.append("未收到预期的容量错误和非零退出码")
    else:
        if exit_code != 0 or log.count("process has finished cleanly") != 3:
            problems.append("三个节点未全部正常退出")
        if len(observer.source_headers) != FRAME_COUNT or observer.decoded_headers != observer.source_headers:
            problems.append("源帧数不符，或解码帧与源帧的数量、时间戳或顺序不一致")
        if data_packets != expected_packets:
            problems.append("H.264 包的 PTS、时间戳或顺序不符")
        if not eos_order_ok:
            problems.append("H.264 结束标记缺失、重复或后面仍有数据包")

    result = {
        "case": case,
        "exit_code": exit_code,
        "source_frames": len(observer.source_headers),
        "encoded_packets": len(data_packets),
        "decoded_frames": len(observer.decoded_headers),
        "eos_order_ok": eos_order_ok,
    }
    for queue in QUEUE_NAMES:
        stats = summarize_queue(samples, queue)
        if stats is None and not overflow_queue:
            problems.append(f"没有{QUEUE_NAMES[queue]}队列采样")
        if queue == slow_queue and stats is not None:
            if stats.peak_oldest_wait_ms < 1000 / FPS:
                problems.append(f"{QUEUE_NAMES[queue]}队列的等待时间未达到一帧间隔")
            if not stats.drained_after_backlog:
                problems.append(f"未采样到{QUEUE_NAMES[queue]}队列积压后恢复为空")
        for field in fields(QueueStats):
            result[f"{queue}_{field.name}"] = None if stats is None else getattr(stats, field.name)
    result["passed"] = not problems
    result["details"] = "；".join(problems)
    return result, samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, help="CSV 和日志目录，默认新建临时目录")
    parser.add_argument("--domain-id", type=int, default=160, help="起始 ROS 域 ID，连续使用五个域")
    args = parser.parse_args()
    max_starting_domain = 232 - len(CASES) + 1
    if not 0 <= args.domain_id <= max_starting_domain:
        parser.error(f"--domain-id 必须在 0 到 {max_starting_domain} 之间")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    output = args.output_dir
    if output is None:
        output = Path(tempfile.mkdtemp(prefix="h264-queue-pressure-"))
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    print(f"报告目录 {output}", flush=True)

    with tempfile.TemporaryDirectory(prefix="queue-pressure-input-") as directory:
        scratch = Path(directory)
        video = scratch / "test.mp4"
        subprocess.run([
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i",
            f"testsrc2=size=162x120:rate={FPS}", "-frames:v", str(FRAME_COUNT),
            "-c:v", "mpeg4", str(video),
        ], check=True)
        delay_library = build_delay_library(scratch)
        results = []
        all_samples = []
        for index, case in enumerate(CASES):
            print(f"运行 {case} …", flush=True)
            result, samples = run_case(
                case, args.domain_id + index, video, delay_library, output / f"{case}.log",
            )
            results.append(result)
            all_samples.extend(samples)
            status = "通过" if result["passed"] else "失败"
            peaks = []
            for queue, name in QUEUE_NAMES.items():
                packets = result[f"{queue}_peak_packets"]
                wait = result[f"{queue}_peak_oldest_wait_ms"]
                if packets is None:
                    peaks.append(f"{name}队列无采样")
                else:
                    peaks.append(f"{name}峰值 {packets} 包/{wait} ms")
            print(
                f"  {status}，退出码 {result['exit_code']}，"
                f"源帧/编码包/解码帧 {result['source_frames']}/"
                f"{result['encoded_packets']}/{result['decoded_frames']}，{'，'.join(peaks)}",
                flush=True,
            )
            if result["details"]:
                print(f"  {result['details']}", flush=True)

    with (output / "runs.csv").open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    with (output / "queue_samples.csv").open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=SAMPLE_FIELDS)
        writer.writeheader()
        writer.writerows(all_samples)
    return 0 if all(result["passed"] for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
