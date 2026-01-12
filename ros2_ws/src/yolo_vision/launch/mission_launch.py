import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration 
from launch_ros.actions import Node

def generate_launch_description():

    qgc_path_arg = DeclareLaunchArgument(
        'qgc_patch',
        default_value='QGroundControl',
        description='Camino al QGC'
    )

    qgc_path_val = LaunchConfiguration('qgc_path')
    
    venv_site_packages = os.path.expanduser('~/ros_yolo_env/lib/python3.12/site-packages')

    sim_pkg_share = get_package_share_directory('simulation_launch_pkg')
    sim_launch_path = os.path.join(sim_pkg_share, 'launch', 'start_simulation_dual.py')


    set_python_path = AppendEnvironmentVariable(
        name='PYTHONPATH',
        value=venv_site_packages,
        separator=':'
    )

    start_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path),
        launch_arguments={
            'qgc_path': qgc_path_val
        }.items()
    )

    joy_logic_node = TimerAction(
        period=10.0, 
        actions=[
            Node(
                package='joy_control',
                executable='joy_offboard',
                name='joy_offboard_control',
                output='screen'
            )
        ]
    )

    yolo_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='yolo_vision',
                executable='yolo_node',
                name='yolo_detector',
                output='screen',
                parameters=[{'use_sim_time':True}]
            )
        ]
    )

    # auto_pilot = TimerAction(
    #     period=10.0,
    #     actions=[
    #         Node(
    #             package='yolo_vision',
    #             executable='auto_pilot',
    #             name='vision_navigation',
    #             output='screen',
    #             parameters=[{'use_sim_time':True}]
    #         )
    #     ]
    # )

    rqt_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/yolo/detections']
    )

    # mavros_node = Node(
    #     package='mavros',
    #     executable='mavros_node',
    #     output='screen',
    #     parameters=[
    #         {'fcu_url': 'udp://:14540@127.0.0.1:14557'}, # Conexión estándar SITL
    #         {'system_id': 1},
    #         {'component_id': 1},
    #         {'target_system_id': 1},
    #         {'target_component_id': 1},
    #     ]
    # )

    return LaunchDescription([
        qgc_path_arg,
        set_python_path,
        start_simulation,
        joy_logic_node,
        yolo_node,
        # auto_pilot,
        # mavros_node,
        rqt_view
    ])