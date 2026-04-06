from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    yolo_node = Node(
        package='vision',
        executable='object_detector',
        name='object',
        output='screen'
    )

    aruco_node = Node(
        package='vision',
        executable='aruco_detector',
        name='aruco',
        output='screen',
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'aruco_dictionary': 'DICT_5X5_1000',
            'marker_size_m': 0.20,
        }]
    )

    yolo_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='yolo_view',
        arguments=['/m1/blue/detections']
    )

    aruco_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='aruco_view',
        arguments=['/aruco/image_annotated']
    )

    return LaunchDescription([
        yolo_node,
        aruco_node,
        yolo_view,
        aruco_view
    ])
