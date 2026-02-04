from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    window_detector = Node(
        package='m1_blue',
        executable='window_detector',
        name='m1_blue_window_detector',
        output='screen'
    )

    rqt_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='m1_blue_view',
        arguments=['/m1/blue/detections']
    )

    return LaunchDescription([
        window_detector,
        rqt_view
    ])

