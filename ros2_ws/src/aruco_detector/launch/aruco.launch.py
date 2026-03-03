from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_node = Node(
        package='aruco_detector',
        executable='detect_aruco_node',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'aruco_dictionary': 'DICT_5X5_1000',
            'marker_size_m': 0.20,
        }]
    )

    rqt_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='aruco_view',
        arguments=['/aruco/image_annotated']
    )

    return LaunchDescription([
        aruco_node,
        rqt_view
    ])