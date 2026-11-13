# ROS 2 FFmpeg Video Pipeline

当前实现了音视频文件读取、解码、H.264 视频编码、视频像素格式转换和音频重采样的基础封装。

当前开发容器运行 ROS 2 Humble。构建：

~~~bash
cd /workspaces/ros2/ffmpeg_pipline_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select video_pipeline
source install/setup.bash
ros2 run video_pipeline video_pipeline_node
~~~

节点启动后按 Ctrl+C 停止。

后续开发使用 FFmpeg。若重建容器，需要重新安装开发依赖：

~~~bash
apt-get update
apt-get install -y --no-install-recommends ffmpeg libavformat-dev libavcodec-dev libavutil-dev libswscale-dev libswresample-dev pkg-config
~~~

## 封装类

- `MediaReader`：使用 `libavformat` 打开文件并读取音频或视频 Packet。
- `MediaDecoder`：使用 `libavcodec` 将音频或视频 Packet 解码成 Frame。
- `VideoEncoder`：使用 `libavcodec` 将视频 Frame 编码为 H.264 Packet。
- `ImageConverter`：使用 `libswscale` 转换视频像素格式。
- `AudioResampler`：使用 `libswresample` 转换采样率、采样格式和声道布局。

头文件位于 `src/video_pipeline/include/video_pipeline/`，实现在 `src/video_pipeline/src/`。封装库目前供本包内部使用，尚未导出为其他 ROS 2 包可直接依赖的库。

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
colcon build --packages-select video_pipeline
source install/setup.bash
ros2 run video_pipeline ffmpeg_video_probe /tmp/video_pipeline_test.mp4
ros2 run video_pipeline ffmpeg_audio_probe /tmp/video_pipeline_test.wav
~~~

视频示例读取视频、解码、转为 RGB24 并统计帧数；音频示例读取音频、解码并转为 48 kHz 双声道 S16。两个示例目前只输出统计信息，不显示画面或播放声音，也没有接入 ROS Topic。
