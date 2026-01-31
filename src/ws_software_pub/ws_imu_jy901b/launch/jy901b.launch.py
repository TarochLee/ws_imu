from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ws_imu_jy901b',
            executable='jy901b_node',
            name='jy901b_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 115200,
                'frame_id': 'imu_link',
                'publish_rate_hz': 100.0,
            }],
            # glog 的 --v=1 可以通过 arguments 传入
            arguments=['--v=0']   # 默认不开 debug raw
        )
    ])
