from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='yolo_node',
            name='yolo_detector',
            output='screen',
            emulate_tty=True
        )
    ])
