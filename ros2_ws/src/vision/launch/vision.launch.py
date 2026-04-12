from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    yolo_node = Node(
        package='vision',
        executable='object_detector',
        name='object',
        output='screen'
    )

    green_detector = Node(
        package='vision',
        executable='green_detector',
        name='green_detector',
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
            'marker_size_m': 0.18,
        }]
    )

    usb_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='landing_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video6',
            'pixel_format': 'mjpeg2rgb',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0
        }]
    )

    landing_detector = Node(
        package='vision',
        executable='down_camera',
        name='landing_detector',
        output='screen'
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

    landing_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='landing_view',
        arguments=['/m4/landing/detections']
    )

    green_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='green_view',
        arguments=['/m1/green/detections']
    )

    return LaunchDescription([
        usb_camera,
        yolo_node,
        green_detector,
        landing_detector,
        aruco_node,
        yolo_view,
        green_view,
        aruco_view,
        landing_view
    ])
