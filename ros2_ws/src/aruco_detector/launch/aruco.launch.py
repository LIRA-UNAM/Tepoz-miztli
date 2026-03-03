from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # ===== REALSENSE CAMERA NODE =====
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,
            'color_width': 1280,
            'color_height': 720,
            'color_fps': 30,
        }]
    )

    # ===== ARUCO DETECTOR NODE =====
    aruco_node = Node(
        package='aruco_detector',
        executable='detect_aruco_node',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'aruco_dictionary': 'DICT_5X5_1000',
            'marker_size_m': 0.20,   #20cm x 20cm
        }]
    )

    return LaunchDescription([
        realsense_node,
        aruco_node
    ])