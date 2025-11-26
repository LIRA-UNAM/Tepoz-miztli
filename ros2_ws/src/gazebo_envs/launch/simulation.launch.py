from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_gazebo_envs = FindPackageShare('gazebo_envs')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    world_path = PathJoinSubstitution([pkg_gazebo_envs, 'worlds', 'my_world.world'])

    return LaunchDescription([

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH', 
            value=PathJoinSubstitution([pkg_gazebo_envs, 'models']),
            separator=':'
        ),

        IncludeLaunchDescription( 
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={ 
                'gz_args': ['-r ', world_path], 
                'on_exit_shutdown': 'True'
            }.items(),
        ),
    ])