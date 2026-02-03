from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('ws_imu_orientation')
    rviz_config = os.path.join(pkg, 'rviz', 'imu_orientation.rviz')

    imu_node = Node(
        package='ws_imu_orientation',
        executable='imu_orientation_node',
        name='imu_orientation_node',
        output='screen',
        parameters=[
            {
                'input_topic': '/imu/jy901b',
                'imu_topic': '/imu/data',
                'marker_topic': '/imu/axes_markers',
                'frame_id': 'imu_link',
                'axis_length': 0.6,
                'axis_radius': 0.03,
                'text_scale': 0.12,
                'publish_markers': True,
            }
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([imu_node, rviz_node])
