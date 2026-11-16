from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
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
    return LaunchDescription([
        DeclareLaunchArgument('input_path', description='容器内的视频文件路径'),
        DeclareLaunchArgument('fps', default_value='30', description='发布和编码帧率，使用正整数'),
        DeclareLaunchArgument('bit_rate', default_value='2000000', description='编码目标码率，单位 bit/s'),
        RegisterEventHandler(OnProcessExit(on_exit=stop_on_failure)),
        Node(
            package='video_pipeline',
            executable='h264_decoder_node',
            output='screen',
            parameters=[{'announce_ready': True}],
        ),
        Node(
            package='video_pipeline',
            executable='h264_encoder_node',
            output='screen',
            parameters=[{
                'fps': ParameterValue(fps, value_type=int),
                'bit_rate': ParameterValue(LaunchConfiguration('bit_rate'), value_type=int),
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
            }],
        ),
    ])
