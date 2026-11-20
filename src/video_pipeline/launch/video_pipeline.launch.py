from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def stop_on_failure(event, context):
    """节点异常退出时结束整条链路，避免其他节点继续等待数据。"""
    if event.returncode != 0 and not context.is_shutdown:
        raise RuntimeError(f'{event.process_name} 异常退出，退出码 {event.returncode}')


def generate_launch_description():
    """启动视频文件源、H.264 编码和 H.264 解码节点。"""
    fps = LaunchConfiguration('fps')
    with_audio = LaunchConfiguration('with_audio')
    return LaunchDescription([
        DeclareLaunchArgument('input_path', description='容器内的视频文件路径'),
        DeclareLaunchArgument('fps', default_value='30', description='发布和编码帧率，使用正整数'),
        DeclareLaunchArgument('bit_rate', default_value='2000000', description='编码目标码率，单位 bit/s'),
        DeclareLaunchArgument(
            'encoder_backend', default_value='auto', description='H.264 编码后端，auto、software 或 nvenc'),
        DeclareLaunchArgument(
            'decoder_backend', default_value='auto', description='H.264 解码后端，auto、software 或 nvdec'),
        DeclareLaunchArgument('output_width', default_value='0', description='编码输出宽度，0 表示使用输入宽度'),
        DeclareLaunchArgument('output_height', default_value='0', description='编码输出高度，0 表示使用输入高度'),
        DeclareLaunchArgument('with_audio', default_value='false', description='编码、解码和同步输入文件的音轨'),
        DeclareLaunchArgument('audio_bit_rate', default_value='64000', description='Opus 目标码率，单位 bit/s'),
        DeclareLaunchArgument(
            'adaptive_bit_rate', default_value='false', description='开启自适应码率'),
        DeclareLaunchArgument('send_queue_max_packets', default_value='120', description='发送队列最大包数'),
        DeclareLaunchArgument('send_queue_max_bytes', default_value='8388608', description='发送队列最大数据字节数'),
        DeclareLaunchArgument('decode_queue_max_packets', default_value='120', description='解码队列最大包数'),
        DeclareLaunchArgument('decode_queue_max_bytes', default_value='8388608', description='解码队列最大数据字节数'),
        RegisterEventHandler(OnProcessExit(on_exit=stop_on_failure)),
        Node(
            package='video_pipeline',
            executable='h264_decoder_node',
            output='screen',
            parameters=[{
                'announce_ready': True,
                'decoder_backend': ParameterValue(LaunchConfiguration('decoder_backend'), value_type=str),
                'queue_max_packets': ParameterValue(
                    LaunchConfiguration('decode_queue_max_packets'), value_type=int),
                'queue_max_bytes': ParameterValue(
                    LaunchConfiguration('decode_queue_max_bytes'), value_type=int),
            }],
        ),
        Node(
            package='video_pipeline',
            executable='h264_encoder_node',
            output='screen',
            parameters=[{
                'fps': ParameterValue(fps, value_type=int),
                'bit_rate': ParameterValue(LaunchConfiguration('bit_rate'), value_type=int),
                'encoder_backend': ParameterValue(LaunchConfiguration('encoder_backend'), value_type=str),
                'output_width': ParameterValue(LaunchConfiguration('output_width'), value_type=int),
                'output_height': ParameterValue(LaunchConfiguration('output_height'), value_type=int),
                'adaptive_bit_rate': ParameterValue(
                    LaunchConfiguration('adaptive_bit_rate'), value_type=bool),
                'queue_max_packets': ParameterValue(
                    LaunchConfiguration('send_queue_max_packets'), value_type=int),
                'queue_max_bytes': ParameterValue(
                    LaunchConfiguration('send_queue_max_bytes'), value_type=int),
            }],
        ),
        Node(
            package='video_pipeline',
            executable='video_file_source_node',
            output='screen',
            parameters=[{
                'input_path': ParameterValue(LaunchConfiguration('input_path'), value_type=str),
                'fps': ParameterValue(fps, value_type=float),
                'wait_for_pipeline': True,
                'media_timestamps': ParameterValue(with_audio, value_type=bool),
            }],
        ),
        Node(
            package='video_pipeline', executable='av_sync_node', output='screen',
            condition=IfCondition(with_audio),
        ),
        Node(
            package='video_pipeline', executable='opus_decoder_node', output='screen',
            condition=IfCondition(with_audio), parameters=[{'announce_ready': True}],
        ),
        Node(
            package='video_pipeline', executable='opus_encoder_node', output='screen',
            condition=IfCondition(with_audio),
            parameters=[{
                'announce_ready': True,
                'bit_rate': ParameterValue(LaunchConfiguration('audio_bit_rate'), value_type=int),
            }],
        ),
        Node(
            package='video_pipeline', executable='audio_file_source_node', output='screen',
            condition=IfCondition(with_audio),
            parameters=[{
                'input_path': ParameterValue(LaunchConfiguration('input_path'), value_type=str),
                'wait_for_av_start': True,
            }],
        ),
    ])
