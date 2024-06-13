# scan_merger_custom/launch/scan_merger_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scan_merger_custom',
            executable='merge_scans',
            name='scan_merger',
            parameters=[{
                'scan_topic1': '/scan1',
                'scan_topic2': '/scan2',
                'merged_scan_topic': '/merged_scan'
            }],
            output='screen'
        )
    ])

