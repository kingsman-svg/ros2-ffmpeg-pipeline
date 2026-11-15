# ROS 2 FFmpeg Video Pipeline

当前实现了音视频文件读取、解码、H.264 视频编码、视频像素格式转换和音频重采样的基础封装。视频发布节点把文件解码为 RGB 图像，H.264 编码节点订阅图像并发布压缩数据。

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
apt-get install -y --no-install-recommends ros-humble-sensor-msgs ros-humble-rosidl-default-generators \
  ffmpeg libavformat-dev libavcodec-dev libavutil-dev libswscale-dev libswresample-dev pkg-config
~~~

## 发布视频文件

在容器里的第一个终端生成测试视频并启动节点：

~~~bash
ffmpeg -y -f lavfi -i 'testsrc2=size=162x120:rate=10:duration=30' \
  -c:v mpeg4 /tmp/video_publisher_demo.mp4
ros2 run video_pipeline video_pipeline_node --ros-args \
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
