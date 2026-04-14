"""
Launch Mision 3: Circuito Gate Blue - Aterrizaje

Despegue, Cruce de Gate Blue, Esquivar columnas, Identiica zona de aterizaje, Landing.

Levanta: 
vision.launch.py
Nodo mision3

Comando de uso en la terminal:
ros2 launch TMR mision3.launch.py
""" 

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_source import PythonLaunchDescriptSource
from launch_ros.actions import Node

def generate_launch_description():
    #Paquetes
    vision_share = get_package_share_directory('vision')
    
    #Subnodos
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptSource(
            os.path.join(vision_share, 'launch',  'vision_launch.py')
        )
    )

    mision3_node = Node(
        package='TMR',
        executable='mision3',
        name='mision3',
        output='screen',
        emulate_tty=True,
    )
 
    return LaunchDescription([
        vision_launch,
        mision3_node,
    ])
