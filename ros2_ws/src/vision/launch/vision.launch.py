from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    vision_node = Node(
        package='vision',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )

    rqt_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='vision_view',
        arguments=['/vision/image']
    )

    return LaunchDescription([
        vision_node,
        rqt_view
    ])
