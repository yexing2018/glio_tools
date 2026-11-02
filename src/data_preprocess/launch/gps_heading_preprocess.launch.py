from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    heading_topic = LaunchConfiguration('heading_topic')
    gps_topic = LaunchConfiguration('gps_topic')
    output_topic = LaunchConfiguration('output_topic')
    sync_tolerance_sec = LaunchConfiguration('sync_tolerance_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'heading_topic', default_value='/imu_orientation'),
        DeclareLaunchArgument('gps_topic', default_value='/fix'),
        DeclareLaunchArgument(
            'output_topic', default_value='/gps_heading_preprocessed'),
        DeclareLaunchArgument('sync_tolerance_sec', default_value='0.03'),
        Node(
            package='data_preprocess',
            executable='gps_heading_preprocess',
            name='gps_heading_preprocess',
            output='screen',
            parameters=[{
                'heading_topic': heading_topic,
                'gps_topic': gps_topic,
                'output_topic': output_topic,
                'sync_tolerance_sec': sync_tolerance_sec,
            }],
        ),
    ])
