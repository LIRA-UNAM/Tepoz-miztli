import os 
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, AppendEnvironmentVariable, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    ros_setup_path = f'/opt/ros/{ros_distro}/setup.bash'
    
    simulation_pkg_share_dir = get_package_share_directory('simulation_launch_pkg')
    
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(simulation_pkg_share_dir))))
    
    repo_root = os.path.dirname(ws_root) 
    

    px4_autopilot_dir = os.path.join(repo_root, 'PX4-Autopilot')
    
    px4_ros_com_ws_path = os.path.join(repo_root, 'px4_ros_com_ws')
    
    px4_ros_com_setup_path = os.path.join(px4_ros_com_ws_path, 'install', 'setup.bash')
    
    drone_ws_setup_path = os.path.join(ws_root, 'install', 'setup.bash')
    
    px4_models_path = os.path.expanduser('~/Tepoz-miztli/PX4-Autopilot/Tools/simulation/gz/models')

    
    qgc_path_arg = DeclareLaunchArgument(
        'qgc_path',
        default_value='QGroundControl',
        description='Path to QGC'
    )
    qgc_path = LaunchConfiguration('qgc_path')

    
    start_sitl_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             f'source {ros_setup_path} && '
             f'source {px4_ros_com_setup_path} && '
             f'make px4_sitl gz_x500_dual_cam_my_world' 
             ],
             cwd=px4_autopilot_dir,
             output='screen',
    )

    start_qgc_cmd = TimerAction(
        period=10.0, 
        actions=[
            ExecuteProcess(
                cmd=[qgc_path], 
                output='screen'
            )
        ]
    )

    start_microxrce_agent_cmd = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     f'source {ros_setup_path} && '
                     f'source {px4_ros_com_setup_path} && '
                     f'MicroXRCEAgent udp4 -p 8888'
                     ],
                output='screen',
            )
        ]
    )

    # start_offboard_control_cmd = TimerAction(
    #     period=30.0,    
    #     actions=[
    #         ExecuteProcess(
    #             cmd=['bash', '-c',
    #                  f'source {ros_setup_path} && '
    #                  f'source {px4_ros_com_setup_path} && '
    #                  f'source {drone_ws_setup_path} && '
    #                  f'ros2 run px4_ros_com offboard_control' 
    #                  ], 
    #             output='screen', 
    #         )
    #     ]
    # )

    joy_driver = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    #Iniciar los modelos del mundo
    set_models = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{px4_models_path}/models:{px4_models_path}/worlds",
        # value=px4_models_path,
        separator=':'
    )
    
    front_rgb_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_front_rgb',
        output='screen',
        arguments=[
            '/world/my_world/model/x500_dual_cam_0/model/front_camera/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        remappings=[
            ('/world/my_world/model/x500_dual_cam_0/model/front_camera/link/camera_link/sensor/IMX214/image', '/front_camera/image_raw')
        ]
    )

    front_depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_front_depth',
        output='screen',
        arguments=[
            '/world/my_world/model/x500_dual_cam_0/model/front_camera/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        remappings=[
            ('/world/my_world/model/x500_dual_cam_0/model/front_camera/link/camera_link/sensor/StereoOV7251/depth_image', '/front_camera/depth_image_raw')
        ]
    )

    down_rgb_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge', 
        name='bridge_down_rgb',
        output='screen',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        remappings=[
            ('/camera/image', '/down_camera/image_raw')
        ]
    )

    down_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_down_info',
        output='screen',
        arguments=[
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        remappings=[
            ('/camera/camera_info', '/down_camera/camera_info')
        ]
    )

    return LaunchDescription([
        set_models,
        qgc_path_arg,
        start_sitl_cmd,
        start_qgc_cmd,
        start_microxrce_agent_cmd,
        #start_offboard_control_cmd,
        joy_driver,
        front_depth_bridge,
        front_rgb_bridge,
        down_info_bridge,
        down_rgb_bridge
    ])