# ROS 2 FFmpeg Video Pipeline

当前实现了音视频文件读取、解码、H.264 视频编码、视频像素格式转换和音频重采样的基础封装。视频文件经发布节点输出 RGB 图像，编码节点将图像压缩为 H.264，解码节点再输出 RGB 图像。

当前开发容器运行 ROS 2 Humble。构建：

~~~bash
cd /workspaces/ros2/ffmpeg_pipline_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to video_pipeline
source install/setup.bash
~~~

若重建容器，需要安装开发依赖：

~~~bash
apt-get update
apt-get install -y --no-install-recommends ros-humble-sensor-msgs \
  ros-humble-rosidl-default-generators ros-humble-ros2launch \
  ffmpeg libavformat-dev libavcodec-dev libavutil-dev libswscale-dev libswresample-dev pkg-config
~~~

## 一键运行视频链路

在容器中生成示例视频，再启动文件源、编码和解码三个节点。

~~~bash
ffmpeg -y -f lavfi -i 'testsrc2=size=162x120:rate=10:duration=30' \
  -c:v mpeg4 /tmp/video_publisher_demo.mp4
ros2 launch video_pipeline video_pipeline.launch.py \
  input_path:=/tmp/video_publisher_demo.mp4 fps:=10 bit_rate:=2000000
~~~

`input_path` 必填。`fps` 是用于发布和编码的正整数帧率，默认 `30`。`bit_rate` 是编码目标码率，默认 `2000000` bit/s。文件源等图像、结束通知和 H.264 链路就绪后才开始发布。视频播完后，三个节点会依次排出缓冲数据并退出。

## 发布视频文件

在容器里的第一个终端生成测试视频并启动节点：

~~~bash
ffmpeg -y -f lavfi -i 'testsrc2=size=162x120:rate=10:duration=30' \
  -c:v mpeg4 /tmp/video_publisher_demo.mp4
ros2 run video_pipeline video_file_source_node --ros-args \
  -p input_path:=/tmp/video_publisher_demo.mp4 -p fps:=10.0
~~~

`input_path` 是容器内的文件路径。`fps` 是固定发布帧率，默认 `30.0`，可以按测试文件的帧率设置。节点默认发布到 `/video/image_raw`，消息类型为 `sensor_msgs/msg/Image`，编码为 `rgb8`。`header.stamp` 记录节点处理这帧图像时的 ROS 时间。文件读完并排空解码器后，节点自动退出。

在同一容器的第二个终端查看话题和消息：

~~~bash
source /opt/ros/humble/setup.bash
source /workspaces/ros2/ffmpeg_pipline_ws/install/setup.bash
ros2 topic list -t
ros2 topic info /video/image_raw --verbose
cat "$(ros2 pkg prefix --share sensor_msgs)/msg/Image.msg"
ros2 topic echo /video/image_raw --once --no-arr
ros2 topic echo /video/image_raw --once --truncate-length 24
ros2 topic hz /video/image_raw
~~~

`Image.msg` 给出消息字段定义，`echo` 查看实际收到的帧。`--no-arr` 省略像素数组，`--truncate-length` 让终端只显示一部分像素字节。使用示例视频时，应看到 `width: 162`、`height: 120`、`encoding: rgb8`、`step: 486`，接收频率约为 10 Hz。`topic hz` 按 Ctrl+C 结束。

## 发布 H.264 数据

先在容器中启动编码节点，再运行上一节的视频发布命令。每个终端都需要加载 ROS 2 和工作空间环境。

~~~bash
source /opt/ros/humble/setup.bash
source /workspaces/ros2/ffmpeg_pipline_ws/install/setup.bash
ros2 run video_pipeline h264_encoder_node --ros-args -p fps:=10 -p bit_rate:=2000000
~~~

编码端的 `fps` 使用整数，示例与视频发布端的 `10.0` 对应。`bit_rate` 单位为 bit/s。编码节点从 `/video/image_raw` 接收固定尺寸的 `rgb8` 图像，将其转为 YUV420P，再通过 `libx264` 编码。输入宽高均须为偶数。图像和编码包使用可靠 QoS。一次运行处理一个文件，编码端需要在源节点发布第一帧前完成订阅。

发布期间，在另一个容器终端查看自定义消息、关键帧标记和带宽。

~~~bash
source /opt/ros/humble/setup.bash
source /workspaces/ros2/ffmpeg_pipline_ws/install/setup.bash
ros2 topic info /video/h264 --verbose
ros2 interface show video_pipeline_msgs/msg/H264Packet
ros2 topic echo /video/h264 --once --no-arr
ros2 topic bw /video/h264
~~~

`/video/h264` 的每条数据消息包含一个 Annex B 编码包及对应输入图像的 ROS 头部。编码时间戳 PTS 从 0 开始，单位为 `1 / fps` 秒。源节点读完文件后，通过 `/video/end_of_stream` 发送输入帧数。编码节点等这些帧处理完，排出缓冲中的编码包，再在 `/video/h264` 发布 `end_of_stream: true` 并退出。结束标记的 `data` 为空。

## 解码 H.264 数据

在容器中启动解码节点，再按上面的步骤依次启动编码节点和视频发布节点。每个终端都需要加载 ROS 2 和工作空间环境。

~~~bash
source /opt/ros/humble/setup.bash
source /workspaces/ros2/ffmpeg_pipline_ws/install/setup.bash
ros2 run video_pipeline h264_decoder_node
~~~

解码节点订阅 `/video/h264`，从关键帧开始解码，将输出帧转成 `rgb8` 并发布到 `/video/decoded_image`。输出类型为 `sensor_msgs/msg/Image`，保留原图像的 ROS 时间戳和 `frame_id`。收到 `end_of_stream` 后，节点先发布解码器中剩余的帧，再打印发布帧数并退出。

在播完前，用另一个已加载环境的终端运行以下命令查看图像和帧率。

~~~bash
ros2 topic info /video/decoded_image --verbose
ros2 topic echo /video/decoded_image --once --no-arr
ros2 topic hz /video/decoded_image
~~~

对照 `/video/image_raw` 和 `/video/decoded_image`，可以检查宽高、帧率和 ROS 时间戳。H.264 采用有损编码，解码图像的像素值可能和源图像不同。

### H.264 发送与解码队列

编码后的数据包先进入发送队列，由发送线程发布到 `/video/h264`。解码节点的订阅回调把收到的包放进解码队列，由解码线程取出并处理。两个应用层队列各自默认最多保留 120 个包和 8 MiB 的 H.264 数据。`ros2 launch` 接受以下参数。

| 参数 | 作用 |
|---|---|
| `send_queue_max_packets` | 发送队列包数上限 |
| `send_queue_max_bytes` | 发送队列数据字节上限 |
| `decode_queue_max_packets` | 解码队列包数上限 |
| `decode_queue_max_bytes` | 解码队列数据字节上限 |

单独用 `ros2 run` 启动编码或解码节点时，通过 `queue_max_packets` 和 `queue_max_bytes` 配置该节点的队列。两个节点每秒打印当前包数、数据字节占用及占用率，以及最老包在队列里的等待时间。统计在应用层入队时开始，取出后即结束。达到任一容量上限且无法接收新包时，节点报错退出。正常结束时，两端都先处理完已入队的数据，再发结束标记或排空解码器。

编码节点每秒还会打印累计输入帧数、发送队列入队与出队包数、发布包数和 H.264 数据字节数。发送线程在数据包的 `publish()` 调用返回后累计包数和字节数，发送结束时打印最终总量。结束标记有单独的发布路径，包数和字节数只统计视频数据。

### 查看解码队列反馈

解码节点每 200 ms 在 `/video/queue_feedback` 发布一份 `H264QueueFeedback`。消息包含当前包数、当前 H.264 数据字节数、两种容量上限和最老包等待时间。反馈使用 `KeepLast(1)`、`BestEffort` 和 `Volatile`。

反馈还带有从解码节点启动起的累计数据。

| 字段 | 统计口径 |
|---|---|
| `sample_elapsed_us` | 采样时的单调时钟经过量，单位为微秒 |
| `total_enqueued_packets`、`total_dequeued_packets` | 成功入队和取出的 H.264 数据包数，结束标记不计入 |
| `total_published_frames` | 成功发布的 RGB 图像帧数 |
| `total_decode_time_us` | FFmpeg 送包、取帧和结束排空调用的累计耗时 |
| `total_convert_time_us` | 像素格式转为 RGB 的累计耗时 |
| `total_publish_time_us` | 构建 ROS 图像消息和调用 `publish()` 的累计耗时 |

三段耗时都使用 `std::chrono::steady_clock`，解码线程处理完一个包或结束排空后汇总。把两次采样的 `sample_elapsed_us` 差值换算成秒，再用计数差除以这段时间，就能得到每秒入队包数、取出包数和发布帧数。

编码节点每秒打印本地发送队列和最新的解码队列反馈。接收后满 1 秒仍没有新样本时，反馈标为过期。超时由编码端的单调时钟计算。

~~~bash
ros2 interface show video_pipeline_msgs/msg/H264QueueFeedback
ros2 topic echo /video/queue_feedback --qos-reliability best_effort
~~~

### 运行 H.264 队列压测

在 Linux 开发容器里完成构建后，在工作空间根目录运行。脚本使用容器已有的 `ffmpeg` 和 C 编译器。

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/queue_pressure_test.py
~~~

脚本生成 80 帧、10 fps 的测试视频，依次跑正常、慢发送、慢解码、反馈中断、发送队列超限和解码队列超限六个场景。慢速场景跳过前 5 次发布，再对指定线程的 18 次发布各延迟 220 ms。脚本检查目标队列积压后排空，并在慢解码场景核对编码端收到的反馈。反馈中断场景跳过前 10 条反馈，丢弃之后的 18 条，检查编码端能否识别过期并重新收到有效反馈。超限场景把对应队列的字节上限设为 1，检查节点是否报出对应错误并使 launch 非零退出。

前四个场景还会检查 80 个 H.264 包及其 PTS、解码图像和源图像的时间戳与顺序，以及 H.264 结束标记是否只在最后出现一次。脚本在终端报告各场景的结果，并打印报告目录，可用 `--output-dir` 指定目录。目录中的 `runs.csv` 记录帧数、退出码、各队列的采样峰值和反馈恢复情况，`queue_samples.csv` 记录每秒的队列包数、字节数、占用率和最老包的等待时间，每个场景还有一份 launch 日志。短时间内退出的超限场景可能没有队列采样。

脚本只在本机 ROS 域内运行，默认依次使用 160 至 165，可用 `--domain-id` 修改起始值。临时测试库只随慢速和反馈中断场景加载。报告中的峰值来自每秒一次的队列日志采样。

## 封装类

- `MediaReader`：使用 `libavformat` 打开文件并读取音频或视频 Packet。
- `MediaDecoder`：使用 `libavcodec` 将音频或视频 Packet 解码成 Frame。
- `VideoEncoder`：使用 `libavcodec` 将视频 Frame 编码为 H.264 Packet。
- `ImageConverter`：使用 `libswscale` 转换视频像素格式。
- `AudioResampler`：使用 `libswresample` 转换采样率、采样格式和声道布局。

在 `video_pipeline` 包内，ROS 2 节点源码位于 `nodes/`，FFmpeg 封装的实现和验证程序位于 `src/`，头文件位于 `include/video_pipeline/`。封装库目前供本包内部使用，尚未导出为其他 ROS 2 包可直接依赖的库。

FFmpeg 封装和示例程序强制使用 C++11。ROS 2 Humble 的 `rclcpp` 头文件依赖 C++14，因此已有的 ROS 节点单独使用 C++14。

`MediaReader` 返回的编码参数由读取器持有，关闭或重新打开后失效；Packet 和 Frame 由调用者分配和释放。编码器、解码器保留 FFmpeg 的 `EAGAIN` 和 `EOF` 返回码，调用时需要先取完输出，并在输入结束后刷新缓冲。每个封装对象应由一个线程顺序使用。

音频重采样输入结束后，需重复以空输入调用 `AudioResampler::convert`，直到返回 0；输出时间戳需由调用者计算。输入音频格式发生变化时，需要重新初始化重采样器。

## 验证文件读取和转换

先在容器中生成临时视频和音频：

~~~bash
ffmpeg -f lavfi -i 'testsrc=size=160x120:rate=10:duration=2' -c:v mpeg4 /tmp/video_pipeline_test.mp4
ffmpeg -f lavfi -i 'sine=frequency=440:sample_rate=44100:duration=1' -c:a pcm_s16le /tmp/video_pipeline_test.wav
~~~

在工作空间根目录编译并运行：

~~~bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to video_pipeline
source install/setup.bash
ros2 run video_pipeline ffmpeg_video_probe /tmp/video_pipeline_test.mp4
ros2 run video_pipeline ffmpeg_audio_probe /tmp/video_pipeline_test.wav
~~~

视频示例读取视频、解码、转为 RGB24 并统计帧数；音频示例读取音频、解码并转为 48 kHz 双声道 S16。两个示例目前只输出统计信息，不显示画面或播放声音，也没有接入 ROS Topic。
