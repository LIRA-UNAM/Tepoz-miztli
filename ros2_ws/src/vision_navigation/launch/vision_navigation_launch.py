import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'vision_navigation'
    executable_name = 'yolo_detector'
    
    realsense_dir = get_package_share_directory('realsense2_camera')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_pointcloud': 'false',
            'enable_sync': 'true',
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30'
        }.items()
    )

    yolo_node = Node(
        package=package_name,
        executable=executable_name,
        name='yolo_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([realsense_launch, yolo_node])